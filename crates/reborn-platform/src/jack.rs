//! The headphone jack (#28 speaker), as ALSA reports it: the card's
//! "Headphone Jack" control (iface CARD), which every ASoC jack creates and
//! which PipeWire and PulseAudio read too. The first card that has one is
//! used, so the same code serves the Y2 (CS43131 detector), the Y1 (a plug
//! switch GPIO) or any other board; the kernel reports the jack only once it
//! has read it. A plug turns a playing speaker off in the kernel already;
//! Reborn follows with the headphone route, and pauses on an unplug as the
//! stock player does.
//!
//! Spoken through the kernel's control interface (`/dev/snd/controlC*`), the
//! interface alsa-lib wraps: the platform crate does not link alsa-lib, and
//! the structures it needs are fixed kernel ABI (`<sound/asound.h>`).
use std::fs::{self, File, OpenOptions};
use std::io::{self, Read};
use std::mem::size_of;
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;
use std::path::{Path, PathBuf};

const CONTROLS: &str = "/dev/snd";
const NAME: &[u8] = b"Headphone Jack";
const IFACE_CARD: i32 = 0;
const EVENT_ELEM: i32 = 0;
const MASK_VALUE: u32 = 1;
/// SNDRV_CTL_EVENT_MASK_REMOVE: the control is gone (it has the value bit too).
const MASK_REMOVE: u32 = u32::MAX;
/// `struct snd_ctl_event`: type, then the element's mask and id.
const EVENT_SIZE: usize = 72;

/// `struct snd_ctl_elem_id`: numid, iface, device, subdevice (u32 each),
/// name[44], index.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct ElemId([u8; 64]);

impl ElemId {
    /// The jack's control by name, as the kernel looks it up (numid 0).
    fn jack() -> Self {
        let mut raw = [0u8; 64];
        raw[4..8].copy_from_slice(&IFACE_CARD.to_ne_bytes());
        raw[16..16 + NAME.len()].copy_from_slice(NAME);
        Self(raw)
    }

    fn numid(&self) -> u32 {
        word(&self.0, 0)
    }
}

/// After the id in `struct snd_ctl_elem_value`: `indirect` and padding to the
/// value union (8 bytes), the union (128 longs), `reserved[128]`.
const VALUE_TAIL: usize = 8 + 128 * size_of::<libc::c_long>() + 128;

/// `struct snd_ctl_elem_value`, aligned for the union's `long long` members.
#[repr(C, align(8))]
struct ElemValue {
    id: ElemId,
    tail: [u8; VALUE_TAIL],
}

/// `_IOWR('U', nr, size)`.
const fn iowr(nr: u64, size: usize) -> u64 {
    (3 << 30) | ((size as u64) << 16) | ((b'U' as u64) << 8) | nr
}
const ELEM_READ: u64 = iowr(0x12, size_of::<ElemValue>());
const SUBSCRIBE_EVENTS: u64 = iowr(0x16, size_of::<libc::c_int>());

fn word(raw: &[u8], at: usize) -> u32 {
    let mut bytes = [0u8; 4];
    bytes.copy_from_slice(&raw[at..at + 4]);
    u32::from_ne_bytes(bytes)
}

pub struct Jack {
    file: File,
    id: ElemId,
}

/// An ioctl whose argument is one structure the kernel reads and writes.
fn ioctl<T>(file: &File, request: u64, arg: &mut T) -> io::Result<()> {
    // SAFETY: `arg` is a live, exclusive `T`, and each request names exactly
    // `size_of::<T>()` bytes (ELEM_READ and SUBSCRIBE_EVENTS above).
    let ret = unsafe { libc::ioctl(file.as_raw_fd(), request as _, arg as *mut T) };
    if ret < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(())
    }
}

/// The control's value now (a boolean: the union's first long); the kernel
/// fills in the id, its numid too.
fn read(file: &File, id: ElemId) -> io::Result<(ElemId, bool)> {
    let mut value = ElemValue {
        id,
        tail: [0; VALUE_TAIL],
    };
    ioctl(file, ELEM_READ, &mut value)?;
    let first = &value.tail[8..8 + size_of::<libc::c_long>()];
    Ok((value.id, first.iter().any(|&b| b != 0)))
}

/// The card control devices, `controlC0` first.
fn controls() -> Vec<PathBuf> {
    let mut found = fs::read_dir(CONTROLS)
        .map(|dir| {
            dir.filter_map(Result::ok)
                .filter(|e| e.file_name().to_string_lossy().starts_with("controlC"))
                .map(|e| e.path())
                .collect::<Vec<_>>()
        })
        .unwrap_or_default();
    found.sort();
    found
}

/// Whether a run of raw `snd_ctl_event`s changed the value of control `numid`.
pub fn value_changed(events: &[u8], numid: u32) -> bool {
    events.as_chunks::<EVENT_SIZE>().0.iter().any(|raw| {
        let mask = word(raw, 4);
        word(raw, 0) as i32 == EVENT_ELEM
            && word(raw, 8) == numid
            && mask != MASK_REMOVE
            && mask & MASK_VALUE != 0
    })
}

impl Jack {
    pub fn open() -> io::Result<Self> {
        controls()
            .iter()
            .find_map(|path| Self::open_card(path).ok())
            .ok_or_else(|| io::Error::from(io::ErrorKind::NotFound))
    }

    fn open_card(path: &Path) -> io::Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(path)?;
        let (id, _) = read(&file, ElemId::jack())?;
        let mut on: libc::c_int = 1;
        ioctl(&file, SUBSCRIBE_EVENTS, &mut on)?;
        Ok(Self { file, id })
    }

    /// Headphones in now; None when the control cannot be read.
    pub fn current(&self) -> Option<bool> {
        read(&self.file, self.id).ok().map(|(_, plugged)| plugged)
    }

    /// The state now, when it changed since the last call.
    pub fn poll(&mut self) -> Option<bool> {
        let mut buffer = [0u8; EVENT_SIZE * 16];
        let mut changed = false;
        loop {
            match self.file.read(&mut buffer) {
                Ok(0) | Err(_) => break,
                Ok(n) => changed |= value_changed(&buffer[..n], self.id.numid()),
            }
        }
        if changed {
            self.current()
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::offset_of;

    fn event(kind: i32, mask: u32, numid: u32) -> Vec<u8> {
        let mut raw = vec![0u8; EVENT_SIZE];
        raw[0..4].copy_from_slice(&kind.to_ne_bytes());
        raw[4..8].copy_from_slice(&mask.to_ne_bytes());
        raw[8..12].copy_from_slice(&numid.to_ne_bytes());
        raw
    }

    #[test]
    fn the_structures_match_the_kernel() {
        // Measured from <sound/asound.h> with the player's arm-linux-gcc and
        // the host's gcc (x86_64): the value union starts at 72 on both.
        assert_eq!(size_of::<ElemId>(), 64);
        assert_eq!(offset_of!(ElemValue, tail) + 8, 72);
        if cfg!(target_pointer_width = "32") {
            assert_eq!(size_of::<ElemValue>(), 712);
            assert_eq!(ELEM_READ, 0xc2c8_5512);
        } else {
            assert_eq!(size_of::<ElemValue>(), 1224);
            assert_eq!(ELEM_READ, 0xc4c8_5512);
        }
        assert_eq!(SUBSCRIBE_EVENTS, 0xc004_5516);
    }

    #[test]
    fn the_jack_is_named_as_asoc_names_it() {
        let id = ElemId::jack();
        assert_eq!(id.numid(), 0);
        assert_eq!(word(&id.0, 4) as i32, IFACE_CARD);
        assert_eq!(&id.0[16..31], b"Headphone Jack\0");
        // The rest of the name, device, subdevice and index stay 0.
        assert!(id.0[31..].iter().chain(&id.0[8..16]).all(|&b| b == 0));
    }

    #[test]
    fn only_a_value_change_of_the_jack_counts() {
        assert!(value_changed(&event(EVENT_ELEM, MASK_VALUE, 7), 7));
        // Another control, an info change, the control removed: no.
        assert!(!value_changed(&event(EVENT_ELEM, MASK_VALUE, 8), 7));
        assert!(!value_changed(&event(EVENT_ELEM, 2, 7), 7));
        assert!(!value_changed(&event(EVENT_ELEM, MASK_REMOVE, 7), 7));
        // Anywhere in a run; a cut-off event at the end is ignored.
        let mut run = event(EVENT_ELEM, 2, 3);
        run.extend(event(EVENT_ELEM, MASK_VALUE | 4, 7));
        assert!(value_changed(&run, 7));
        assert!(!value_changed(&run[..EVENT_SIZE + 11], 7));
    }
}
