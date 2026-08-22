//! Serial ports, per platform.
//!
//! The contract is deliberately small — read, write, flush, and set how long a
//! read may block — because everything above it is protocol, and protocol
//! should not know which operating system it is running on.
//!
//! One of those four is not an ordinary knob. See [`SerialPort::set_read_timeout`].

use std::sync::{Mutex, MutexGuard};
use std::time::Duration;

#[cfg(unix)]
mod unix;
#[cfg(windows)]
mod windows;

#[cfg(unix)]
pub use unix::Port;
#[cfg(windows)]
pub use windows::Port;

/// A serial port.
///
/// The Go version expressed flushing and timeout tuning as optional interfaces
/// a port might or might not implement, discovered by a type assertion at the
/// call site. Here they are ordinary methods with defaults that do nothing, so
/// a caller can always ask and a port that cannot oblige simply does not.
pub trait SerialPort: Send {
    /// Reads up to `buf.len()` bytes. A return of `Ok(0)` means no data was
    /// available within the current read timeout, which is not an error.
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize>;

    /// Writes the whole buffer.
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize>;

    /// Discards whatever is sitting in the driver's buffers, in both
    /// directions.
    fn flush_buffers(&mut self) -> std::io::Result<()> {
        Ok(())
    }

    /// Sets how long a single read may block waiting for data.
    ///
    /// This is the most consequential line in the crate, and it is worth
    /// knowing why. The default on Windows is a *total* read timeout: every
    /// read that finds an empty buffer costs the full timeout, whether or not
    /// the answer arrives a microsecond later. Opened at the conventional 10 ms
    /// and driven by a Dynamixel loop, that put a hard ceiling of about 67
    /// cycles per second on the bus — measured, not estimated. Setting it to
    /// 1 ms took the same loop to 890.
    ///
    /// The right value depends entirely on what is on the other end. A
    /// Dynamixel packet is worth waiting ten milliseconds for. A CAN adapter
    /// answering a 200 Hz control loop has five milliseconds for the whole
    /// round trip, and a read that blocks for ten is the single largest cost in
    /// the cycle.
    fn set_read_timeout(&mut self, _timeout: Duration) -> std::io::Result<()> {
        Ok(())
    }
}

/// A serial device found on this machine.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortInfo {
    /// What [`open`] expects: `COM7` on Windows, `/dev/ttyACM0` on Linux,
    /// `/dev/cu.usbmodem…` on macOS.
    pub path: String,

    /// Whatever the OS reveals about the device — a driver name, a by-id
    /// symlink, or empty when nothing is available.
    pub description: String,

    /// A heuristic: true when the name looks like one of the USB-CAN adapters
    /// this framework knows how to drive. It only ranks the list for the
    /// operator, and is never used to decide what to open.
    pub is_can_adapter: bool,
}

/// Opens a port at the given baud rate.
pub fn open(path: &str, baud: u32) -> std::io::Result<Port> {
    Port::open(path, baud)
}

/// Every serial port on this machine, CAN adapters first.
///
/// A platform that cannot enumerate returns an empty list rather than an error,
/// so a caller can always fall back to an explicitly configured path.
pub fn list() -> std::io::Result<Vec<PortInfo>> {
    let mut ports = enumerate()?;
    ports.sort_by(|a, b| {
        b.is_can_adapter
            .cmp(&a.is_can_adapter)
            .then_with(|| a.path.cmp(&b.path))
    });
    Ok(ports)
}

#[cfg(any(windows, unix))]
use self::platform_enumerate as enumerate;

#[cfg(not(any(windows, unix)))]
fn enumerate() -> std::io::Result<Vec<PortInfo>> {
    Ok(Vec::new())
}

#[cfg(windows)]
use windows::enumerate as platform_enumerate;

#[cfg(unix)]
use unix::enumerate as platform_enumerate;

/// Recognises the USB-CAN adapters this framework can drive, so the operator
/// sees them at the top of the list.
///
/// It matches on whatever text the platform exposes. On Windows that is only
/// `COM7`, so the hint never fires there and the list stays in port order.
pub(crate) fn looks_like_can_adapter(text: &str) -> bool {
    let lower = text.to_ascii_lowercase();
    ["fdcanusb", "mjbots", "canable", "slcan", "gs_usb"]
        .iter()
        .any(|hint| lower.contains(hint))
}

/// One port, used by several devices — a Dynamixel daisy chain, where a dozen
/// servos answer on the same two wires.
///
/// A Protocol 2.0 exchange is flush, write, then read until a packet arrives.
/// If two servos' exchanges interleave, each read consumes the other's reply
/// and both time out. Locking the individual calls would not help; the whole
/// exchange has to be held together, which is why the guard is the only way in.
/// Cloning one hands out another handle to the *same* port, which is what a
/// daisy chain needs: one port opened once, and one driver per servo id on it.
#[derive(Clone)]
pub struct SharedPort {
    port: std::sync::Arc<Mutex<Box<dyn SerialPort>>>,
}

impl SharedPort {
    pub fn new(port: Box<dyn SerialPort>) -> Self {
        Self {
            port: std::sync::Arc::new(Mutex::new(port)),
        }
    }

    /// Claims the port for one transaction.
    pub fn lock(&self) -> MutexGuard<'_, Box<dyn SerialPort>> {
        self.port
            .lock()
            .expect("serial port poisoned by a panicking thread")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adapter_names_are_recognised_whatever_their_case() {
        assert!(looks_like_can_adapter("mjbots fdcanusb"));
        assert!(looks_like_can_adapter("usb-MJBOTS_fdcanusb-if00"));
        assert!(looks_like_can_adapter("CANable v2"));
        assert!(!looks_like_can_adapter("COM7"));
        assert!(!looks_like_can_adapter("/dev/ttyUSB0"));
    }

    /// The list is ordered for the operator's eye, not for the machine.
    #[test]
    fn can_adapters_sort_ahead_of_everything_else() {
        let mut ports = [
            PortInfo {
                path: "/dev/ttyUSB1".into(),
                description: String::new(),
                is_can_adapter: false,
            },
            PortInfo {
                path: "/dev/ttyACM0".into(),
                description: "mjbots fdcanusb".into(),
                is_can_adapter: true,
            },
            PortInfo {
                path: "/dev/ttyUSB0".into(),
                description: String::new(),
                is_can_adapter: false,
            },
        ];
        ports.sort_by(|a, b| {
            b.is_can_adapter
                .cmp(&a.is_can_adapter)
                .then_with(|| a.path.cmp(&b.path))
        });
        let paths: Vec<&str> = ports.iter().map(|p| p.path.as_str()).collect();
        assert_eq!(paths, ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyUSB1"]);
    }

    /// Two handles on one port are two ways into the same lock, not two ports.
    /// A daisy chain depends on that: if cloning duplicated the port, two
    /// servos' transactions would interleave and each would read the other's
    /// reply.
    #[test]
    fn cloning_a_shared_port_shares_the_same_lock() {
        struct Counter(std::sync::Arc<std::sync::atomic::AtomicUsize>);
        impl SerialPort for Counter {
            fn read(&mut self, _buf: &mut [u8]) -> std::io::Result<usize> {
                Ok(0)
            }
            fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
                self.0.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                Ok(buf.len())
            }
        }

        let writes = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let first = SharedPort::new(Box::new(Counter(writes.clone())));
        let second = first.clone();

        first.lock().write(b"a").unwrap();
        second.lock().write(b"b").unwrap();
        assert_eq!(writes.load(std::sync::atomic::Ordering::Relaxed), 2);
    }

    /// A port that cannot honour a request says so by doing nothing, rather
    /// than by not having the method.
    #[test]
    fn an_untunable_port_accepts_the_request_and_ignores_it() {
        struct Plain;
        impl SerialPort for Plain {
            fn read(&mut self, _buf: &mut [u8]) -> std::io::Result<usize> {
                Ok(0)
            }
            fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
                Ok(buf.len())
            }
        }

        let mut port = Plain;
        assert!(port.set_read_timeout(Duration::from_millis(1)).is_ok());
        assert!(port.flush_buffers().is_ok());
    }
}
