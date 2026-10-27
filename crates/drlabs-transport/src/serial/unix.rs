//! Serial ports on Linux and macOS, through termios.

use std::io;
use std::os::unix::ffi::OsStrExt;
use std::time::Duration;

use super::{looks_like_can_adapter, PortInfo, SerialPort};

/// An open serial file descriptor.
pub struct Port {
    fd: libc::c_int,
}

// The descriptor is owned exclusively by this struct and every call through it
// takes `&mut self`.
unsafe impl Send for Port {}

impl Port {
    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        let c_path = std::ffi::CString::new(std::ffi::OsStr::new(path).as_bytes())
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "path contains a NUL"))?;

        // O_NOCTTY keeps the port from becoming this process's controlling
        // terminal, which would route stray input to it. O_NONBLOCK matters
        // twice: it stops the open itself from blocking on a device that has
        // not asserted DCD — the usual reason a macOS open hangs forever — and
        // it makes every later read return at once rather than waiting.
        let fd = unsafe {
            libc::open(
                c_path.as_ptr(),
                libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK,
            )
        };
        if fd < 0 {
            let error = io::Error::last_os_error();
            if cfg!(target_os = "macos") {
                return Err(io::Error::new(
                    error.kind(),
                    format!(
                        "open {path}: {error} (on macOS, try /dev/cu.* rather than /dev/tty.*)"
                    ),
                ));
            }
            return Err(error);
        }

        let mut port = Port { fd };
        if let Err(error) = port.set_params(baud) {
            return Err(error);
        }
        Ok(port)
    }

    fn set_params(&mut self, baud: u32) -> io::Result<()> {
        let mut term: libc::termios = unsafe { std::mem::zeroed() };
        if unsafe { libc::tcgetattr(self.fd, &mut term) } != 0 {
            return Err(io::Error::last_os_error());
        }

        // 8N1.
        term.c_cflag &= !libc::CSIZE;
        term.c_cflag |= libc::CS8;
        term.c_cflag &= !libc::PARENB;
        term.c_cflag &= !libc::CSTOPB;
        // Read, and do not care about modem control lines: a USB adapter has
        // none, and waiting for carrier detect on one waits forever.
        term.c_cflag |= libc::CREAD | libc::CLOCAL;

        // Raw. Every one of these flags exists to make a terminal pleasant for
        // a human, and every one of them corrupts a binary protocol: ICRNL
        // rewrites 0x0d as 0x0a mid-packet, IXON eats 0x11 and 0x13 as flow
        // control, OPOST edits what is written on the way out.
        term.c_lflag &= !(libc::ICANON | libc::ECHO | libc::ECHOE | libc::ISIG);
        term.c_oflag &= !libc::OPOST;
        term.c_iflag &= !(libc::IXON | libc::IXOFF | libc::IXANY);
        term.c_iflag &= !(libc::IGNBRK
            | libc::BRKINT
            | libc::PARMRK
            | libc::ISTRIP
            | libc::INLCR
            | libc::IGNCR
            | libc::ICRNL);

        // Non-blocking reads: return whatever is there, immediately.
        term.c_cc[libc::VMIN] = 0;
        term.c_cc[libc::VTIME] = 0;

        let speed = baud_constant(baud).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "{baud} baud is not a rate termios offers on Linux (up to 4000000 in the \
                     standard steps); a Dynamixel at 4.5, 6 or 10.5 Mbaud needs the BOTHER \
                     path through termios2, which this build does not have yet"
                ),
            )
        })?;
        if unsafe { libc::cfsetispeed(&mut term, speed) } != 0 {
            return Err(io::Error::last_os_error());
        }
        if unsafe { libc::cfsetospeed(&mut term, speed) } != 0 {
            return Err(io::Error::last_os_error());
        }

        if unsafe { libc::tcsetattr(self.fd, libc::TCSANOW, &term) } != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }
}

impl SerialPort for Port {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let n = unsafe { libc::read(self.fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        if n < 0 {
            let error = io::Error::last_os_error();
            // The port is non-blocking, so "nothing to read" arrives as an
            // error code. It is the ordinary case, not a failure.
            if error.kind() == io::ErrorKind::WouldBlock {
                return Ok(0);
            }
            return Err(error);
        }
        Ok(n as usize)
    }

    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let n = unsafe { libc::write(self.fd, buf.as_ptr() as *const libc::c_void, buf.len()) };
        if n < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(n as usize)
    }

    fn flush_buffers(&mut self) -> io::Result<()> {
        if unsafe { libc::tcflush(self.fd, libc::TCIOFLUSH) } != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }

    /// Accepted and ignored, deliberately.
    ///
    /// The Windows implementation needs this because a read there blocks for a
    /// timeout the caller has to name, and naming it badly costs an order of
    /// magnitude in loop rate. Here the descriptor is opened non-blocking with
    /// `VMIN`/`VTIME` at zero, so a read already returns at once with whatever
    /// has arrived, and there is nothing to tune. The alternative knob termios
    /// offers, `VTIME`, is measured in *tenths of a second* — a hundred times
    /// coarser than the millisecond that matters here.
    fn set_read_timeout(&mut self, _timeout: Duration) -> io::Result<()> {
        Ok(())
    }
}

impl Drop for Port {
    fn drop(&mut self) {
        unsafe {
            libc::close(self.fd);
        }
    }
}

/// The termios speed constant for a baud rate.
///
/// Linux takes an enumerated constant that happens not to equal the rate;
/// macOS takes the rate itself. A rate Linux has no constant for is `None`,
/// and the open fails saying so. It used to fall back to 115200 — "wrong but
/// talks to something" — which on a chain described at 3 Mbaud is a bus
/// where every servo is silent and nothing says why.
#[cfg(target_os = "linux")]
fn baud_constant(baud: u32) -> Option<libc::speed_t> {
    Some(match baud {
        9600 => libc::B9600,
        19200 => libc::B19200,
        38400 => libc::B38400,
        57600 => libc::B57600,
        115200 => libc::B115200,
        230400 => libc::B230400,
        460800 => libc::B460800,
        500000 => libc::B500000,
        576000 => libc::B576000,
        921600 => libc::B921600,
        1000000 => libc::B1000000,
        2000000 => libc::B2000000,
        3000000 => libc::B3000000,
        4000000 => libc::B4000000,
        _ => return None,
    })
}

#[cfg(not(target_os = "linux"))]
fn baud_constant(baud: u32) -> Option<libc::speed_t> {
    Some(baud as libc::speed_t)
}

/// The device nodes a USB serial adapter shows up as.
///
/// `ttyUSB*` is an FTDI or CP210x style bridge, `ttyACM*` a CDC device such as
/// the fdcanusb or the OpenRB-150. On macOS only the callout side is listed:
/// the same device appears as both `/dev/tty.*`, which blocks until DCD is
/// asserted, and `/dev/cu.*`, which does not.
#[cfg(target_os = "linux")]
const DEVICE_PATTERNS: &[&str] = &["/dev/ttyUSB", "/dev/ttyACM"];

#[cfg(target_os = "macos")]
const DEVICE_PATTERNS: &[&str] = &["/dev/cu.usbmodem", "/dev/cu.usbserial", "/dev/cu.SLAB"];

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
const DEVICE_PATTERNS: &[&str] = &[];

/// Enumerates character devices that can plausibly be a serial port.
///
/// `/dev/serial/by-id` is preferred when it exists, because those names carry
/// the manufacturer and serial number — the only way to tell two identical
/// adapters apart across reboots. The plain device nodes are listed as well,
/// since the by-id tree is a udev convention rather than a guarantee.
pub fn enumerate() -> io::Result<Vec<PortInfo>> {
    let mut ports: Vec<PortInfo> = Vec::new();
    let mut seen: Vec<std::path::PathBuf> = Vec::new();

    let mut add = |path: std::path::PathBuf, description: String| {
        let resolved = std::fs::canonicalize(&path).unwrap_or_else(|_| path.clone());
        if seen.contains(&resolved) {
            return;
        }
        seen.push(resolved);
        let path = path.to_string_lossy().into_owned();
        ports.push(PortInfo {
            is_can_adapter: looks_like_can_adapter(&format!("{path} {description}")),
            path,
            description,
        });
    };

    if let Ok(entries) = std::fs::read_dir("/dev/serial/by-id") {
        for entry in entries.flatten() {
            let name = entry.file_name().to_string_lossy().into_owned();
            add(entry.path(), name);
        }
    }

    if let Ok(entries) = std::fs::read_dir("/dev") {
        for entry in entries.flatten() {
            let path = entry.path();
            let text = path.to_string_lossy();
            if DEVICE_PATTERNS
                .iter()
                .any(|prefix| text.starts_with(prefix))
            {
                add(path, String::new());
            }
        }
    }

    Ok(ports)
}
