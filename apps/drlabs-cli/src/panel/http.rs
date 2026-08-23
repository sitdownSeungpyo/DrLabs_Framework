//! A very small HTTP/1.1 server.
//!
//! Deliberately small, and deliberately here rather than from a crate. This
//! serves three static files and a handful of JSON endpoints, to a browser on
//! the same machine, for the length of a bench session. A web framework in a
//! robot binary would be more code, more to keep current, and more surface than
//! the job has.
//!
//! What it is *not* is a general web server. It binds loopback by default, it
//! does not do TLS, keep-alive, chunked bodies, or anything a hostile client
//! might send. If this ever needs to face a network, replace it rather than
//! extend it.

use std::collections::BTreeMap;
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};

/// One request, as much of it as this server cares about.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Request {
    pub method: String,
    pub path: String,
    pub body: String,
}

/// What to send back.
pub struct Response {
    pub status: u16,
    pub content_type: &'static str,
    pub body: Vec<u8>,
}

impl Response {
    pub fn html(body: &str) -> Self {
        Self {
            status: 200,
            content_type: "text/html; charset=utf-8",
            body: body.as_bytes().to_vec(),
        }
    }

    pub fn javascript(body: &str) -> Self {
        Self {
            status: 200,
            content_type: "application/javascript; charset=utf-8",
            body: body.as_bytes().to_vec(),
        }
    }

    pub fn css(body: &str) -> Self {
        Self {
            status: 200,
            content_type: "text/css; charset=utf-8",
            body: body.as_bytes().to_vec(),
        }
    }

    pub fn json(body: String) -> Self {
        Self {
            status: 200,
            content_type: "application/json",
            body: body.into_bytes(),
        }
    }

    /// A refusal the panel can show the operator.
    ///
    /// 409 rather than 400: the request was well formed, the machine said no.
    /// A UI that cannot tell those apart shows "bad request" when the real
    /// answer is "that joint is not armed".
    pub fn refused(reason: &str) -> Self {
        Self {
            status: 409,
            content_type: "application/json",
            body: format!("{{\"error\":{}}}", quote(reason)).into_bytes(),
        }
    }

    pub fn not_found() -> Self {
        Self {
            status: 404,
            content_type: "text/plain; charset=utf-8",
            body: b"not found".to_vec(),
        }
    }
}

/// Escapes a string as a JSON value.
///
/// The panel shows the framework's own error messages, and those contain
/// quotes, newlines and — in joint space — non-ASCII. A message mangled on the
/// way to the screen is worse than none.
pub fn quote(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    out.push('"');
    for character in text.chars() {
        match character {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

/// Reads one request from a connection.
pub fn read_request(stream: &mut TcpStream) -> std::io::Result<Option<Request>> {
    let mut reader = BufReader::new(stream.try_clone()?);

    let mut line = String::new();
    if reader.read_line(&mut line)? == 0 {
        return Ok(None);
    }
    let mut parts = line.split_whitespace();
    let method = parts.next().unwrap_or_default().to_string();
    let path = parts.next().unwrap_or_default().to_string();

    let mut length = 0usize;
    loop {
        let mut header = String::new();
        if reader.read_line(&mut header)? == 0 {
            break;
        }
        let header = header.trim_end();
        if header.is_empty() {
            break;
        }
        if let Some((name, value)) = header.split_once(':') {
            if name.eq_ignore_ascii_case("content-length") {
                length = value.trim().parse().unwrap_or(0);
            }
        }
    }

    // A body larger than any pose this panel sends is not a pose.
    if length > 64 * 1024 {
        return Ok(Some(Request {
            method,
            path,
            body: String::new(),
        }));
    }

    let mut body = vec![0u8; length];
    if length > 0 {
        reader.read_exact(&mut body)?;
    }

    Ok(Some(Request {
        method,
        path,
        body: String::from_utf8_lossy(&body).into_owned(),
    }))
}

pub fn write_response(stream: &mut TcpStream, response: &Response) -> std::io::Result<()> {
    let reason = match response.status {
        200 => "OK",
        404 => "Not Found",
        409 => "Conflict",
        _ => "Error",
    };
    write!(
        stream,
        "HTTP/1.1 {} {reason}\r\nContent-Type: {}\r\nContent-Length: {}\r\nConnection: close\r\n\
         Cache-Control: no-store\r\n\r\n",
        response.status,
        response.content_type,
        response.body.len()
    )?;
    stream.write_all(&response.body)?;
    stream.flush()
}

/// Serves until the process ends.
pub fn serve<F>(listener: TcpListener, handle: F)
where
    F: Fn(&Request) -> Response + Send + Sync + 'static,
{
    let handle = std::sync::Arc::new(handle);
    for stream in listener.incoming() {
        let Ok(mut stream) = stream else { continue };
        let handle = handle.clone();
        std::thread::spawn(move || {
            if let Ok(Some(request)) = read_request(&mut stream) {
                let response = handle(&request);
                let _ = write_response(&mut stream, &response);
            }
        });
    }
}

/// Reads a flat `{"name": number}` object.
///
/// Anything that does not fit that shape is rejected rather than half-read: a
/// target that is not a number must not quietly become zero, which would
/// command the joint to its origin.
pub fn parse_pose(body: &str) -> Result<BTreeMap<String, f64>, String> {
    let pose: BTreeMap<String, f64> =
        serde_json::from_str(body).map_err(|error| error.to_string())?;
    for (name, value) in &pose {
        if name.is_empty() {
            return Err("a pose entry has no joint name".into());
        }
        if !value.is_finite() {
            return Err(format!("{name:?} was given a non-finite target"));
        }
    }
    Ok(pose)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_pose_reads_as_names_and_numbers() {
        let pose = parse_pose(r#"{"r_sh_p": 0.25, "l_sh_p": -1.5}"#).unwrap();
        assert_eq!(pose["r_sh_p"], 0.25);
        assert_eq!(pose["l_sh_p"], -1.5);
        assert!(parse_pose("{}").unwrap().is_empty());
    }

    /// A target that is not a number must not become one. Rounding a mistyped
    /// value to zero would command the joint to its origin.
    #[test]
    fn a_target_that_is_not_a_number_is_refused() {
        assert!(parse_pose(r#"{"j": "0.25"}"#).is_err());
        assert!(parse_pose(r#"{"j": }"#).is_err());
        assert!(parse_pose(r#"{"j": NaN}"#).is_err());
        assert!(parse_pose("not json").is_err());
    }

    #[test]
    fn a_pose_entry_without_a_name_is_refused() {
        assert!(parse_pose(r#"{"": 1.0}"#).is_err());
    }

    /// A non-finite target would reach the physics and poison everything
    /// downstream of it.
    #[test]
    fn a_non_finite_target_is_refused() {
        // JSON has no NaN literal, so this is the shape a client can actually
        // send: a number too large to be finite as an f64.
        assert!(parse_pose(r#"{"j": 1e400}"#).is_err());
    }

    /// The panel shows the framework's own messages, and those contain quotes
    /// and non-ASCII. A message mangled on the way to the screen is worse than
    /// none.
    #[test]
    fn a_message_survives_being_quoted() {
        assert_eq!(quote(r#"joint "j1" refused"#), r#""joint \"j1\" refused""#);
        assert_eq!(quote("line\nbreak"), r#""line\nbreak""#);
        // Joint-space messages are full of these.
        assert_eq!(quote("0.05 rad · ±1.4"), "\"0.05 rad · ±1.4\"");
    }

    /// A well-formed request the machine says no to is a conflict, not a bad
    /// request. A UI that cannot tell those apart shows "bad request" when the
    /// real answer is "that joint is not armed".
    #[test]
    fn a_refusal_carries_its_reason_and_the_right_status() {
        let response = Response::refused("the axis is not armed");
        assert_eq!(response.status, 409);
        let body = String::from_utf8(response.body).unwrap();
        assert!(body.contains("the axis is not armed"), "{body}");
    }
}
