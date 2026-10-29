//! Serial ports on Linux and macOS, through termios.
//!
//! The two agree on everything except how a baud rate is set, and that one
//! difference is the whole reason this file has an OS-specific module in it.
//! Linux takes an enumerated constant through `cfsetspeed`. macOS takes the
//! rate itself, but only through the `IOSSIOSPEED` ioctl — handing 1 000 000
//! to `cfsetspeed` compiles, returns success from `tcgetattr`, and leaves the
//! adapter at whatever it was doing before. A Dynamixel chain opened that way
//! is a bus where every servo is silent and nothing says why.

use std::io;
use std::os::unix::ffi::OsStrExt;
use std::time::{Duration, Instant};

use super::{looks_like_can_adapter, PortInfo, SerialPort};

/// How long a write may stall before it is called a failure.
///
/// A non-blocking descriptor refuses bytes while the driver's output buffer is
/// full, and that buffer is small on macOS: Apple's CDC and FTDI drivers hold a
/// few kilobytes where Linux holds a page or more. A Dynamixel sync write to
/// fifteen servos is a few hundred bytes, so the wait is normally microseconds.
/// Two hundred milliseconds is an adapter that has stopped draining — unplugged,
/// or a device that has stopped asserting flow control — and the error is worth
/// more than a loop that waits forever.
const WRITE_TIMEOUT: Duration = Duration::from_millis(200);

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

    /// Wraps a descriptor that is already open and already non-blocking.
    ///
    /// For tests, which need a `Port` over a pipe to exercise the write path
    /// without a serial device. Not offered to callers: a `Port` that was not
    /// configured by [`Port::open`] is not a serial port.
    #[cfg(test)]
    fn from_raw_fd(fd: libc::c_int) -> Self {
        Port { fd }
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

        speed::into_termios(&mut term, baud)?;

        if unsafe { libc::tcsetattr(self.fd, libc::TCSANOW, &term) } != 0 {
            return Err(io::Error::last_os_error());
        }

        // Must come after tcsetattr: on macOS the attributes call resets the
        // rate to what termios knows, which is not this one.
        speed::after_attributes(self.fd, baud)
    }

    /// Blocks until the descriptor will take more bytes, or `deadline` passes.
    fn wait_writable(&self, deadline: Instant) -> io::Result<()> {
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!(
                        "write stalled for {} ms: the adapter is not draining its buffer",
                        WRITE_TIMEOUT.as_millis()
                    ),
                ));
            }
            let mut fds = libc::pollfd {
                fd: self.fd,
                events: libc::POLLOUT,
                revents: 0,
            };
            // Round up, so a sub-millisecond remainder is one more poll rather
            // than a busy loop against a zero timeout.
            let timeout_ms = remaining.as_millis().max(1).min(i32::MAX as u128) as libc::c_int;
            let ready = unsafe { libc::poll(&mut fds, 1, timeout_ms) };
            if ready > 0 {
                return Ok(());
            }
            if ready < 0 {
                let error = io::Error::last_os_error();
                if error.kind() != io::ErrorKind::Interrupted {
                    return Err(error);
                }
            }
        }
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

    /// Writes the whole buffer, as the trait promises.
    ///
    /// A single `write(2)` on a non-blocking descriptor may take part of the
    /// buffer, or none of it, when the driver is full. The callers — a protocol
    /// packet, a sync write to a chain — do not check the count, and should not
    /// have to: half a Dynamixel packet on the wire is a CRC error at every
    /// servo, and the symptom is a chain that "sometimes" answers.
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let deadline = Instant::now() + WRITE_TIMEOUT;
        let mut written = 0usize;
        while written < buf.len() {
            let rest = &buf[written..];
            let n =
                unsafe { libc::write(self.fd, rest.as_ptr() as *const libc::c_void, rest.len()) };
            if n < 0 {
                let error = io::Error::last_os_error();
                match error.kind() {
                    io::ErrorKind::WouldBlock => self.wait_writable(deadline)?,
                    io::ErrorKind::Interrupted => {}
                    _ => return Err(error),
                }
                continue;
            }
            if n == 0 {
                return Err(io::Error::new(
                    io::ErrorKind::WriteZero,
                    "the port accepted no bytes",
                ));
            }
            written += n as usize;
        }
        Ok(written)
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

/// Setting the line rate, which is the one thing Linux and macOS do differently.
///
/// Two steps, because the two systems want the rate at different moments:
/// [`into_termios`](speed::into_termios) edits the attribute block before it is
/// applied, and [`after_attributes`](speed::after_attributes) runs once it has
/// been. Each OS uses one and leaves the other empty.
#[cfg(target_os = "linux")]
mod speed {
    use std::io;

    /// Linux encodes rates as constants that happen not to equal the rate. A
    /// rate with no constant is refused, and the open fails saying so. It used
    /// to fall back to 115200 — "wrong but talks to something" — which on a
    /// chain described at 3 Mbaud is a bus where every servo is silent.
    pub fn into_termios(term: &mut libc::termios, baud: u32) -> io::Result<()> {
        let constant = match baud {
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
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!(
                        "{baud} baud is not a rate termios offers on Linux (up to 4000000 in the \
                         standard steps); a Dynamixel at 4.5, 6 or 10.5 Mbaud needs the BOTHER \
                         path through termios2, which this build does not have yet"
                    ),
                ))
            }
        };
        if unsafe { libc::cfsetispeed(term, constant) } != 0 {
            return Err(io::Error::last_os_error());
        }
        if unsafe { libc::cfsetospeed(term, constant) } != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }

    pub fn after_attributes(_fd: libc::c_int, _baud: u32) -> io::Result<()> {
        Ok(())
    }
}

#[cfg(target_os = "macos")]
mod speed {
    use std::io;

    /// `IOSSIOSPEED` from `<IOKit/serial/ioss.h>`: `_IOW('T', 2, speed_t)`.
    ///
    /// Written out rather than imported because the `libc` crate does not
    /// carry it. The encoding is BSD's: `IOC_IN` (0x8000_0000), the parameter
    /// size in bits 16..29 (`speed_t` is a `c_ulong`, eight bytes), the group
    /// `'T'` (0x54) in bits 8..15 and the number 2 at the bottom. On a 32-bit
    /// build the size field would be 4 and the value 0x8004_5402; nothing
    /// here is built 32-bit.
    const IOSSIOSPEED: libc::c_ulong = 0x8008_5402;

    /// Leaves the rate alone: the ioctl below sets it, and whatever
    /// `cfsetspeed` were given would be ignored by the driver anyway for any
    /// rate above the classic ones.
    pub fn into_termios(_term: &mut libc::termios, _baud: u32) -> io::Result<()> {
        Ok(())
    }

    /// Any rate the adapter's driver accepts, including the 1, 2 and 3 Mbaud a
    /// Dynamixel chain runs at. Apple's own FTDI and CDC drivers take this
    /// call; a refusal is the driver saying the rate is outside what the
    /// hardware can clock.
    pub fn after_attributes(fd: libc::c_int, baud: u32) -> io::Result<()> {
        let speed: libc::speed_t = baud as libc::speed_t;
        if unsafe { libc::ioctl(fd, IOSSIOSPEED, &speed) } != 0 {
            let error = io::Error::last_os_error();
            return Err(io::Error::new(
                error.kind(),
                format!("IOSSIOSPEED refused {baud} baud: {error} (this adapter's driver does not clock that rate)"),
            ));
        }
        Ok(())
    }
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
mod speed {
    use std::io;

    pub fn into_termios(_term: &mut libc::termios, _baud: u32) -> io::Result<()> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "setting a serial baud rate is only implemented for Linux and macOS",
        ))
    }

    pub fn after_attributes(_fd: libc::c_int, _baud: u32) -> io::Result<()> {
        Ok(())
    }
}

/// Whether a name under `/dev` is plausibly a serial adapter.
///
/// Linux: `ttyUSB*` is an FTDI or CP210x style bridge, `ttyACM*` a CDC device
/// such as the fdcanusb or the OpenRB-150.
#[cfg(target_os = "linux")]
fn is_serial_candidate(name: &str) -> bool {
    name.starts_with("ttyUSB") || name.starts_with("ttyACM")
}

/// macOS: every device appears twice, as `tty.*`, which blocks until DCD is
/// asserted, and `cu.*`, which does not. Only the callout side is listed, and
/// only the names a USB serial driver produces: `cu.usbmodem…` (CDC: the
/// fdcanusb, the OpenRB-150), `cu.usbserial-…` (FTDI: the U2D2), `cu.SLAB_USBtoUART`
/// (CP210x), `cu.wchusbserial…` (CH340), `cu.PL2303-…` (Prolific). An allow
/// list rather than "everything under `cu.`" because Bluetooth creates a port
/// for every paired device under that device's own name — this Mac has a
/// `cu.--` — and no deny list can know them all.
#[cfg(target_os = "macos")]
fn is_serial_candidate(name: &str) -> bool {
    const USB_DRIVERS: &[&str] = &[
        "cu.usb",
        "cu.SLAB_USBtoUART",
        "cu.wchusbserial",
        "cu.PL2303",
    ];
    USB_DRIVERS.iter().any(|prefix| name.starts_with(prefix))
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
fn is_serial_candidate(_name: &str) -> bool {
    false
}

/// Enumerates character devices that can plausibly be a serial port.
///
/// `/dev/serial/by-id` is preferred when it exists, because those names carry
/// the manufacturer and serial number — the only way to tell two identical
/// adapters apart across reboots. The plain device nodes are listed as well,
/// since the by-id tree is a udev convention rather than a guarantee. macOS has
/// no such tree and exposes no product string through the device node, so the
/// description there is empty and the CAN-adapter hint never fires; the same
/// is true of Windows.
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
            let name = entry.file_name().to_string_lossy().into_owned();
            if is_serial_candidate(&name) {
                add(entry.path(), String::new());
            }
        }
    }

    Ok(ports)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A non-blocking pipe, so the write end behaves like a serial descriptor
    /// whose driver buffer fills: 64 KiB on both Linux and macOS, after which
    /// `write` returns `EAGAIN`.
    fn nonblocking_pipe() -> (libc::c_int, libc::c_int) {
        let mut fds = [0 as libc::c_int; 2];
        assert_eq!(unsafe { libc::pipe(fds.as_mut_ptr()) }, 0);
        let flags = unsafe { libc::fcntl(fds[1], libc::F_GETFL) };
        assert!(flags >= 0);
        assert_eq!(
            unsafe { libc::fcntl(fds[1], libc::F_SETFL, flags | libc::O_NONBLOCK) },
            0
        );
        (fds[0], fds[1])
    }

    fn payload(len: usize) -> Vec<u8> {
        (0..len).map(|i| (i % 251) as u8).collect()
    }

    /// Larger than any driver buffer. The reader drains slowly, so the writer
    /// sees `EAGAIN` many times over and has to wait each time.
    #[test]
    fn a_write_larger_than_the_buffer_arrives_whole() {
        let (read_end, write_end) = nonblocking_pipe();
        let mut port = Port::from_raw_fd(write_end);
        let sent = payload(300_000);
        let expected = sent.len();

        let reader = std::thread::spawn(move || {
            let mut got = Vec::with_capacity(expected);
            let mut chunk = [0u8; 4096];
            while got.len() < expected {
                let n = unsafe {
                    libc::read(
                        read_end,
                        chunk.as_mut_ptr() as *mut libc::c_void,
                        chunk.len(),
                    )
                };
                assert!(n > 0, "reader saw {n}");
                got.extend_from_slice(&chunk[..n as usize]);
                // Slow enough that the pipe fills and the writer has to wait.
                std::thread::sleep(Duration::from_micros(200));
            }
            unsafe { libc::close(read_end) };
            got
        });

        let written = port.write(&sent).expect("write");
        assert_eq!(written, sent.len());
        let got = reader.join().unwrap();
        assert_eq!(got, sent);
    }

    /// Nobody reading is an adapter that has stopped draining. The write must
    /// fail, and it must fail in about the configured time rather than hang.
    #[test]
    fn a_write_nobody_drains_times_out_instead_of_hanging() {
        let (read_end, write_end) = nonblocking_pipe();
        let mut port = Port::from_raw_fd(write_end);

        let started = Instant::now();
        let error = port.write(&payload(300_000)).unwrap_err();
        let elapsed = started.elapsed();

        assert_eq!(error.kind(), io::ErrorKind::TimedOut, "{error}");
        assert!(
            elapsed >= WRITE_TIMEOUT && elapsed < WRITE_TIMEOUT * 5,
            "timed out after {elapsed:?}"
        );
        unsafe { libc::close(read_end) };
    }

    #[cfg(target_os = "macos")]
    #[test]
    fn every_usb_bridge_is_a_candidate_and_the_system_ports_are_not() {
        for name in [
            "cu.usbmodem1234501",    // fdcanusb, OpenRB-150 (CDC)
            "cu.usbserial-D30AXXXX", // U2D2, FT232H (FTDI)
            "cu.SLAB_USBtoUART",     // CP210x
            "cu.wchusbserial14230",  // CH340
            "cu.PL2303-00001014",    // Prolific
        ] {
            assert!(is_serial_candidate(name), "{name}");
        }
        for name in [
            "cu.Bluetooth-Incoming-Port",
            "cu.debug-console",
            "cu.wlan-debug",
            "cu.--",               // a paired Bluetooth device, as seen on this Mac
            "cu.AirPods-Pro",      // another one
            "tty.usbmodem1234501", // the dial-in side blocks on DCD
            "null",
        ] {
            assert!(!is_serial_candidate(name), "{name}");
        }
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn only_usb_serial_nodes_are_candidates() {
        assert!(is_serial_candidate("ttyUSB0"));
        assert!(is_serial_candidate("ttyACM3"));
        assert!(!is_serial_candidate("ttyS0"));
        assert!(!is_serial_candidate("tty"));
        assert!(!is_serial_candidate("null"));
    }
}
