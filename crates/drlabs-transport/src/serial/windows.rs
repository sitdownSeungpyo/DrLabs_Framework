//! Windows COM ports, through the Win32 communications API.

use std::io;
use std::os::windows::ffi::OsStrExt;
use std::time::Duration;

use windows_sys::Win32::Devices::Communication::{
    GetCommState, PurgeComm, SetCommState, SetCommTimeouts, SetupComm, COMMTIMEOUTS, DCB, NOPARITY,
    ONESTOPBIT, PURGE_RXABORT, PURGE_RXCLEAR, PURGE_TXABORT, PURGE_TXCLEAR,
};
use windows_sys::Win32::Foundation::{
    CloseHandle, GetLastError, ERROR_INSUFFICIENT_BUFFER, HANDLE, INVALID_HANDLE_VALUE,
};
use windows_sys::Win32::Storage::FileSystem::{
    CreateFileW, QueryDosDeviceW, ReadFile, WriteFile, FILE_GENERIC_READ, FILE_GENERIC_WRITE,
    OPEN_EXISTING,
};

use super::{looks_like_can_adapter, PortInfo, SerialPort};

/// The sentinel the Windows timeout structure uses to mean "no limit of this
/// kind".
const MAX_DWORD: u32 = 0xFFFF_FFFF;

/// An open COM port.
pub struct Port {
    handle: HANDLE,
}

// The handle is owned exclusively by this struct and every call through it
// takes `&mut self`, so moving one between threads is sound.
unsafe impl Send for Port {}

impl Port {
    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        // COM10 and above are only reachable through the device namespace; the
        // prefix costs nothing for the low-numbered ones, so it always goes on.
        let wide: Vec<u16> = std::ffi::OsStr::new(&format!("\\\\.\\{path}"))
            .encode_wide()
            .chain(std::iter::once(0))
            .collect();

        let handle = unsafe {
            CreateFileW(
                wide.as_ptr(),
                FILE_GENERIC_READ | FILE_GENERIC_WRITE,
                0, // no sharing: a serial port has one owner
                std::ptr::null(),
                OPEN_EXISTING,
                0,
                0,
            )
        };
        if handle == INVALID_HANDLE_VALUE || handle == 0 {
            return Err(io::Error::last_os_error());
        }

        let mut port = Port { handle };
        port.set_params(baud)?;
        port.set_default_timeouts()?;
        Ok(port)
    }

    fn set_params(&mut self, baud: u32) -> io::Result<()> {
        let mut dcb: DCB = unsafe { std::mem::zeroed() };
        dcb.DCBlength = std::mem::size_of::<DCB>() as u32;

        if unsafe { GetCommState(self.handle, &mut dcb) } == 0 {
            return Err(io::Error::last_os_error());
        }

        dcb.BaudRate = baud;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        // Bit 0 is fBinary, which Windows requires to be set. The remaining
        // flow-control bits stay clear: these adapters do not use handshaking,
        // and an accidental RTS/CTS setting shows up as a port that opens and
        // then never delivers a byte.
        dcb._bitfield = 1;

        if unsafe { SetCommState(self.handle, &dcb) } == 0 {
            return Err(io::Error::last_os_error());
        }
        if unsafe { SetupComm(self.handle, 4096, 4096) } == 0 {
            return Err(io::Error::last_os_error());
        }
        self.flush_buffers()
    }

    /// The timeouts a port opens with: a plain total timeout of 10 ms.
    ///
    /// Conservative, and wrong for a control loop — which is exactly what
    /// [`SerialPort::set_read_timeout`] is for. It stays as the default because
    /// a caller that has not thought about it is better served waiting than
    /// spinning.
    fn set_default_timeouts(&mut self) -> io::Result<()> {
        let timeouts = COMMTIMEOUTS {
            ReadIntervalTimeout: 0,
            ReadTotalTimeoutMultiplier: 0,
            ReadTotalTimeoutConstant: 10,
            WriteTotalTimeoutMultiplier: 0,
            WriteTotalTimeoutConstant: 10,
        };
        if unsafe { SetCommTimeouts(self.handle, &timeouts) } == 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }
}

impl SerialPort for Port {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut read = 0u32;
        let ok = unsafe {
            ReadFile(
                self.handle,
                buf.as_mut_ptr(),
                buf.len() as u32,
                &mut read,
                std::ptr::null_mut(),
            )
        };
        if ok == 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(read as usize)
    }

    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut written = 0u32;
        let ok = unsafe {
            WriteFile(
                self.handle,
                buf.as_ptr(),
                buf.len() as u32,
                &mut written,
                std::ptr::null_mut(),
            )
        };
        if ok == 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(written as usize)
    }

    fn flush_buffers(&mut self) -> io::Result<()> {
        let ok = unsafe {
            PurgeComm(
                self.handle,
                PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR,
            )
        };
        if ok == 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }

    /// The combination below is the documented way to say "return as soon as
    /// any byte arrives, but wait no longer than this": the interval and the
    /// multiplier both at their sentinel, with the constant carrying the limit.
    ///
    /// The alternative — a plain total timeout, which is what the port opens
    /// with — makes every read that finds an empty buffer cost the whole
    /// timeout. That is the difference between 67 and 890 cycles per second on
    /// a Dynamixel bus, measured on the bench.
    fn set_read_timeout(&mut self, timeout: Duration) -> io::Result<()> {
        // Sub-millisecond is not expressible here, and rounding down to zero
        // would mean "return immediately whatever happens", turning every read
        // into a poll that never waits for a reply.
        let milliseconds = (timeout.as_millis() as u32).max(1);

        let timeouts = COMMTIMEOUTS {
            ReadIntervalTimeout: MAX_DWORD,
            ReadTotalTimeoutMultiplier: MAX_DWORD,
            ReadTotalTimeoutConstant: milliseconds,
            WriteTotalTimeoutMultiplier: 0,
            WriteTotalTimeoutConstant: 10,
        };
        if unsafe { SetCommTimeouts(self.handle, &timeouts) } == 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }
}

impl Drop for Port {
    fn drop(&mut self) {
        // Closing is the whole of shutdown, and there is nobody left to tell if
        // it fails. This is one place Rust is plainly better than the Go
        // version: a port cannot be left open by an early return.
        unsafe {
            CloseHandle(self.handle);
        }
    }
}

/// Enumerates COM ports through the DOS device namespace.
///
/// `QueryDosDeviceW` with a null device name returns every symbolic link in the
/// namespace as a block of NUL-terminated UTF-16 strings. The `COM*` entries in
/// it are exactly the ports that can be opened — which is what we want, because
/// the registry's SERIALCOMM key keeps stale entries for devices that have been
/// unplugged.
pub fn enumerate() -> io::Result<Vec<PortInfo>> {
    // The namespace is large and its size is not knowable in advance; grow
    // until the call stops asking for more room.
    let mut size = 8192usize;
    for _ in 0..6 {
        let mut buf = vec![0u16; size];
        let written =
            unsafe { QueryDosDeviceW(std::ptr::null(), buf.as_mut_ptr(), size as u32) } as usize;
        if written == 0 {
            if unsafe { GetLastError() } == ERROR_INSUFFICIENT_BUFFER {
                size *= 4;
                continue;
            }
            // Nothing enumerable — not a condition the caller can act on.
            return Ok(Vec::new());
        }
        return Ok(parse_dos_devices(&buf[..written]));
    }
    Ok(Vec::new())
}

fn parse_dos_devices(buf: &[u16]) -> Vec<PortInfo> {
    buf.split(|&unit| unit == 0)
        .filter(|chunk| !chunk.is_empty())
        .map(String::from_utf16_lossy)
        .filter(|name| is_com_port(name))
        .map(|name| PortInfo {
            is_can_adapter: looks_like_can_adapter(&name),
            path: name,
            description: String::new(),
        })
        .collect()
}

/// Matches `COM` followed by digits only, so that entries such as `COMPOSITE`
/// are not mistaken for ports.
fn is_com_port(name: &str) -> bool {
    match name.strip_prefix("COM") {
        Some(digits) => !digits.is_empty() && digits.bytes().all(|b| b.is_ascii_digit()),
        None => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_com_followed_by_digits_is_a_port() {
        assert!(is_com_port("COM1"));
        assert!(is_com_port("COM17"));
        assert!(!is_com_port("COM"));
        assert!(!is_com_port("COMPOSITE"));
        assert!(!is_com_port("COM_1"));
        assert!(!is_com_port("LPT1"));
    }

    #[test]
    fn the_device_namespace_is_split_on_its_nul_terminators() {
        let mut buf: Vec<u16> = Vec::new();
        for name in ["COM3", "COMPOSITE", "COM11", "Nul"] {
            buf.extend(name.encode_utf16());
            buf.push(0);
        }
        buf.push(0); // the block's own terminator

        let ports = parse_dos_devices(&buf);
        let paths: Vec<&str> = ports.iter().map(|p| p.path.as_str()).collect();
        assert_eq!(paths, ["COM3", "COM11"]);
    }
}
