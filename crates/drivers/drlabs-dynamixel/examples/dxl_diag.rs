//! Bench diagnostic: dump the registers that decide whether a goal is
//! accepted, and compare an individual write with a sync write on one servo.
//!
//!   dxl_diag <port> <baud> dump <ids>
//!   dxl_diag <port> <baud> test <id> [delta_counts] [all_ids]
//!   dxl_diag <port> <baud> fastprobe <ids> [len]
//!   dxl_diag <port> <baud> syncstress <ids> <chunk> <seconds> [arm] [len] [hz=N] [fast] [write]
//!
//! `test` writes goal = present + delta three ways and reads Goal Position
//! back after each: individual write (0x03), sync write with one id, sync
//! write with every id (the others commanded to their own present position).
//! It restores goal = present at the end. Torque is left as found.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use drlabs_dynamixel::Driver;
use drlabs_transport::serial::SharedPort;

const REGS: &[(&str, u16, u16)] = &[
    ("drive", 10, 1),
    ("opmode", 11, 1),
    ("vel_lim", 44, 4),
    ("max_pos", 48, 4),
    ("min_pos", 52, 4),
    ("torque", 64, 1),
    ("srl", 68, 1),
    ("hwerr", 70, 1),
    ("wdog", 98, 1),
    ("p_acc", 108, 4),
    ("p_vel", 112, 4),
    ("goal", 116, 4),
    ("moving", 122, 1),
    ("present", 132, 4),
];

fn parse_ids(s: &str) -> Vec<u8> {
    s.split(',').filter_map(|x| x.trim().parse().ok()).collect()
}

fn val(data: &[u8]) -> i64 {
    match data.len() {
        1 => data[0] as i64,
        2 => i16::from_le_bytes([data[0], data[1]]) as i64,
        4 => i32::from_le_bytes([data[0], data[1], data[2], data[3]]) as i64,
        _ => -1,
    }
}

fn read_i32(d: &Driver, id: u8, addr: u16) -> Result<i32, String> {
    d.read(id, addr, 4)
        .map(|b| val(&b) as i32)
        .map_err(|e| e.to_string())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 5 {
        eprintln!("usage: dxl_diag <port> <baud> dump <ids> | test <id> [delta] [all_ids] | fastprobe <ids> [len] | syncstress <ids> <chunk> <seconds> [arm] [len] [hz=N] [fast] [write] | release | wdtest | raw | rdt");
        std::process::exit(2);
    }
    let baud: u32 = args[2].parse().expect("baud");
    let port = drlabs_transport::serial::open(&args[1], baud).expect("open port");
    let driver = Arc::new(Driver::new(SharedPort::new(Box::new(port))));
    driver.tune_read_timeout();

    match args[3].as_str() {
        "dump" => {
            let ids = parse_ids(&args[4]);
            print!("{:>3}", "id");
            for (name, _, _) in REGS {
                print!("{:>9}", name);
            }
            println!();
            for id in ids {
                print!("{id:>3}");
                for (_, addr, len) in REGS {
                    match driver.read(id, *addr, *len) {
                        Ok(b) => print!("{:>9}", val(&b)),
                        Err(_) => print!("{:>9}", "ERR"),
                    }
                }
                println!();
            }
        }
        "test" => {
            let id: u8 = args[4].parse().expect("id");
            let delta: i32 = args.get(5).and_then(|s| s.parse().ok()).unwrap_or(100);
            let all: Vec<u8> = args
                .get(6)
                .map(|s| parse_ids(s))
                .unwrap_or_else(|| vec![id]);

            let arm = args.get(7).map(|s| s == "arm").unwrap_or(false);
            if arm {
                driver.write_u32(id, 112, 30).expect("profile velocity");
                driver.write_u8(id, 64, 1).expect("torque on");
                std::thread::sleep(Duration::from_millis(100));
            }
            let present = read_i32(&driver, id, 132).expect("present");
            let goal0 = read_i32(&driver, id, 116).expect("goal");
            let torque = driver.read(id, 64, 1).map(|b| b[0]).unwrap_or(255);
            println!("id {id}: present={present} goal={goal0} torque={torque}");
            let target = present + delta;

            // 1. individual write
            let r = driver.write_u32(id, 116, target as u32);
            std::thread::sleep(Duration::from_millis(20));
            let back = read_i32(&driver, id, 116);
            std::thread::sleep(Duration::from_millis(400));
            println!("[1] individual write {target}: result={r:?} readback={back:?} present_after={:?}", read_i32(&driver, id, 132));
            let _ = driver.write_u32(id, 116, present as u32);
            std::thread::sleep(Duration::from_millis(300));

            // 2. sync write, one id
            let mut v = BTreeMap::new();
            v.insert(id, (target as u32).to_le_bytes().to_vec());
            let r = driver.sync_write(116, 4, &v);
            std::thread::sleep(Duration::from_millis(20));
            let back = read_i32(&driver, id, 116);
            std::thread::sleep(Duration::from_millis(400));
            println!("[2] sync write (1 id) {target}: result={r:?} readback={back:?} present_after={:?}", read_i32(&driver, id, 132));
            let _ = driver.write_u32(id, 116, present as u32);
            std::thread::sleep(Duration::from_millis(300));

            // 3. sync write, every id: others get their own present
            let mut v = BTreeMap::new();
            for &other in &all {
                if other == id {
                    v.insert(id, (target as u32).to_le_bytes().to_vec());
                } else if let Ok(p) = read_i32(&driver, other, 132) {
                    v.insert(other, (p as u32).to_le_bytes().to_vec());
                } else {
                    println!("   id {other}: present read failed, left out");
                }
            }
            let r = driver.sync_write(116, 4, &v);
            std::thread::sleep(Duration::from_millis(20));
            let back = read_i32(&driver, id, 116);
            println!("[3] sync write ({} ids) {target}: result={r:?} readback={back:?}", v.len());
            std::thread::sleep(Duration::from_millis(300));
            let now = read_i32(&driver, id, 132);
            println!("    present after 300 ms: {now:?}");
            let _ = driver.write_u32(id, 116, present as u32);
            std::thread::sleep(Duration::from_millis(300));
            println!("restored goal={present}, present now {:?}", read_i32(&driver, id, 132));
            if arm {
                driver.write_u8(id, 64, 0).expect("torque off");
                println!("torque off");
            }
        }
        // release <ids>: torque off and clear the bus watchdog on every id.
        "release" => {
            for id in parse_ids(&args[4]) {
                let t = driver.write_u8(id, 64, 0);
                let w = driver.write_u8(id, 98, 0);
                println!("id {id}: torque off {t:?}, watchdog clear {w:?}");
            }
        }
        // wdtest <id> <kind>: arm the watchdog at 400 ms, then for 2 s send
        // only packets of `kind` (syncread | syncwrite | read | none) every
        // 100 ms, and report whether the watchdog latched.
        "wdtest" => {
            let id: u8 = args[4].parse().expect("id");
            let kind = args.get(5).map(String::as_str).unwrap_or("none");
            driver.write_u8(id, 98, 0).expect("clear");
            driver.write_u8(id, 64, 1).expect("torque on");
            driver.write_u8(id, 98, 20).expect("arm watchdog 400 ms");
            let present = read_i32(&driver, id, 132).expect("present");
            let started = std::time::Instant::now();
            while started.elapsed() < Duration::from_secs(2) {
                match kind {
                    "syncread" => {
                        let _ = driver.sync_read(132, 4, &[id]);
                    }
                    "syncwrite" => {
                        let mut v = BTreeMap::new();
                        v.insert(id, (present as u32).to_le_bytes().to_vec());
                        let _ = driver.sync_write(116, 4, &v);
                    }
                    "read" => {
                        let _ = driver.read(id, 132, 4);
                    }
                    _ => {}
                }
                std::thread::sleep(Duration::from_millis(100));
            }
            let wd = driver.read(id, 98, 1).map(|b| b[0]);
            println!("id {id} after 2 s of {kind} every 100 ms: watchdog reg98 = {wd:?} (20 = still armed, 255 = latched)");
            // Does a latched servo take a goal? Try present+50 via sync write and read back.
            let mut v = BTreeMap::new();
            v.insert(id, ((present + 50) as u32).to_le_bytes().to_vec());
            let _ = driver.sync_write(116, 4, &v);
            std::thread::sleep(Duration::from_millis(50));
            println!("   sync write goal {} -> reg116 = {:?}", present + 50, read_i32(&driver, id, 116));
            let r = driver.write_u32(id, 116, (present + 50) as u32);
            println!("   individual write goal {} -> {:?}, reg116 = {:?}", present + 50, r, read_i32(&driver, id, 116));
            std::thread::sleep(Duration::from_millis(300));
            let _ = driver.write_u8(id, 98, 0);
            let _ = driver.write_u32(id, 116, present as u32);
            std::thread::sleep(Duration::from_millis(300));
            let _ = driver.write_u8(id, 64, 0);
            println!("   released: present {:?}", read_i32(&driver, id, 132));
        }
        // fastprobe <ids>: which servos answer a fast sync read (0x8A)? One
        // per servo, alone, so a refusal is unambiguous; then every servo
        // that said yes together, timed, to see the combined reply on a real
        // chain. Firmware version (reg 6) is printed beside each answer so
        // the threshold can be read off the table.
        "fastprobe" => {
            let ids = parse_ids(&args[4]);
            let len: u16 = args.get(5).and_then(|s| s.parse().ok()).unwrap_or(10);
            let mut yes = Vec::new();
            println!("{:>3} {:>6} {:>4}  fast sync read", "id", "model", "fw");
            for &id in &ids {
                let model = driver
                    .ping(id)
                    .map(|m| m.to_string())
                    .unwrap_or_else(|_| "ERR".into());
                let fw = driver
                    .read(id, 6, 1)
                    .map(|b| b[0].to_string())
                    .unwrap_or_else(|_| "ERR".into());
                let t0 = std::time::Instant::now();
                let verdict = match driver.fast_sync_read(126, len, &[id]) {
                    Ok((a, _)) if a.contains_key(&id) => {
                        yes.push(id);
                        format!("yes ({:.2} ms)", t0.elapsed().as_secs_f64() * 1e3)
                    }
                    Ok(_) => "no reply segment".to_string(),
                    Err(e) => format!("no: {e}"),
                };
                println!("{id:>3} {model:>6} {fw:>4}  {verdict}");
            }
            if yes.len() > 1 {
                println!("all {} together, {len} bytes each:", yes.len());
                for _ in 0..5 {
                    let t0 = std::time::Instant::now();
                    match driver.fast_sync_read(126, len, &yes) {
                        Ok((a, _)) => println!(
                            "  {} of {} answered in {:.2} ms",
                            a.len(),
                            yes.len(),
                            t0.elapsed().as_secs_f64() * 1e3
                        ),
                        Err(e) => println!("  error: {e}"),
                    }
                    std::thread::sleep(Duration::from_millis(20));
                }
            }
        }
        // syncstress <ids> <chunk> <seconds> [arm] [len] [hz=N] [fast] [write]:
        // cycle reads of the fast block the way the chain does, in chunks of
        // `chunk` (plain sync read) or as one fast sync read, at `hz` (10 by
        // default), optionally with a goal=present sync write each cycle as
        // the chain sends, and report cycle time, achieved rate and which
        // ids go missing, how often, and for how long in a row.
        "syncstress" => {
            let ids = parse_ids(&args[4]);
            let chunk: usize = args.get(5).and_then(|s| s.parse().ok()).unwrap_or(8);
            let seconds: u64 = args.get(6).and_then(|s| s.parse().ok()).unwrap_or(20);
            let mut arm = false;
            let mut fast = false;
            let mut write = false;
            let mut len: u16 = 10;
            let mut hz: f64 = 10.0;
            for token in args.iter().skip(7) {
                match token.as_str() {
                    "arm" => arm = true,
                    "fast" => fast = true,
                    "write" => write = true,
                    t if t.starts_with("hz=") => hz = t[3..].parse().expect("hz=N"),
                    t => len = t.parse().expect("len"),
                }
            }
            let period = Duration::from_secs_f64(1.0 / hz);
            let mut goals: BTreeMap<u8, Vec<u8>> = BTreeMap::new();
            for &id in &ids {
                let p = read_i32(&driver, id, 132).expect("present");
                goals.insert(id, (p as u32).to_le_bytes().to_vec());
                if arm {
                    driver.write_u32(id, 116, p as u32).expect("goal=present");
                    driver.write_u8(id, 64, 1).expect("torque on");
                }
            }
            let mut misses: BTreeMap<u8, (u32, u32, u32)> = BTreeMap::new(); // total, run, max run
            let mut cycles = 0u32;
            let mut errors = 0u32;
            let mut late = 0u32;
            let mut took_us: Vec<u32> = Vec::with_capacity((hz * seconds as f64) as usize + 1);
            let started = std::time::Instant::now();
            let mut next = started;
            while started.elapsed() < Duration::from_secs(seconds) {
                let t0 = std::time::Instant::now();
                if write {
                    if let Err(e) = driver.sync_write(116, 4, &goals) {
                        println!("cycle {cycles}: sync write error {e}");
                    }
                }
                let mut got: Vec<u8> = Vec::new();
                if fast {
                    match driver.fast_sync_read(126, len, &ids) {
                        Ok((a, _)) => got.extend(a.keys()),
                        Err(e) => {
                            errors += 1;
                            if errors <= 10 {
                                println!("cycle {cycles}: fast sync read error {e}");
                            }
                        }
                    }
                } else {
                    for c in ids.chunks(chunk) {
                        match driver.sync_read(126, len, c) {
                            Ok(a) => got.extend(a.keys()),
                            Err(e) => {
                                errors += 1;
                                if errors <= 10 {
                                    println!("cycle {cycles}: sync read error {e}");
                                }
                            }
                        }
                    }
                }
                let took = t0.elapsed();
                took_us.push(took.as_micros().min(u32::MAX as u128) as u32);
                if took > period {
                    late += 1;
                }
                for &id in &ids {
                    let e = misses.entry(id).or_default();
                    if got.contains(&id) {
                        e.1 = 0;
                    } else {
                        e.0 += 1;
                        e.1 += 1;
                        e.2 = e.2.max(e.1);
                    }
                }
                cycles += 1;
                next += period;
                if let Some(d) = next.checked_duration_since(std::time::Instant::now()) {
                    std::thread::sleep(d);
                }
            }
            let elapsed = started.elapsed().as_secs_f64();
            took_us.sort_unstable();
            let pct = |p: f64| took_us[((took_us.len() - 1) as f64 * p) as usize] as f64 / 1e3;
            println!(
                "{} {} ids x {len} B{}: {cycles} cycles in {elapsed:.1} s = {:.1} Hz asked {hz:.0}",
                if fast {
                    "fast sync read".to_string()
                } else {
                    format!("sync read chunk {chunk}")
                },
                ids.len(),
                if write { " + goal sync write" } else { "" },
                cycles as f64 / elapsed
            );
            println!(
                "  cycle p50 {:.2} ms  p95 {:.2} ms  p99 {:.2} ms  max {:.2} ms; {late} over the {:.2} ms period, {errors} transaction errors",
                pct(0.50),
                pct(0.95),
                pct(0.99),
                pct(1.0),
                period.as_secs_f64() * 1e3
            );
            let total_missed: u32 = misses.values().map(|m| m.0).sum();
            println!(
                "  readings missed {total_missed} of {} ({:.2} %)",
                cycles as u64 * ids.len() as u64,
                total_missed as f64 * 100.0 / (cycles as f64 * ids.len() as f64)
            );
            for (id, (total, _, max_run)) in &misses {
                if *total > 0 {
                    println!(
                        "  id {id:>2}: missed {total} ({:.1} %), longest run {max_run}",
                        *total as f64 * 100.0 / cycles as f64
                    );
                }
            }
            if arm {
                for &id in &ids {
                    let _ = driver.write_u8(id, 64, 0);
                }
                println!("torque off");
            }
        }
        // raw <ids> <len>: one sync read, and everything that came back on
        // the wire for 50 ms — valid packets, CRC failures, stray bytes.
        "raw" => {
            let ids = parse_ids(&args[4]);
            let len: u16 = args.get(5).and_then(|s| s.parse().ok()).unwrap_or(1);
            let mut params = Vec::new();
            params.extend_from_slice(&126u16.to_le_bytes());
            params.extend_from_slice(&len.to_le_bytes());
            params.extend_from_slice(&ids);
            let packet = drlabs_dynamixel::protocol::build_packet(0xFE, 0x82, &params);
            let port2 = drlabs_transport::serial::open(&args[1], baud);
            // The Driver owns the port; reopen is refused, so borrow through a
            // second SharedPort clone is not possible. Use the driver's port
            // via a private read: drop the driver first.
            drop(port2);
            drop(driver);
            let mut port = drlabs_transport::serial::open(&args[1], baud).expect("reopen");
            use drlabs_transport::serial::SerialPort;
            let _ = port.set_read_timeout(Duration::from_millis(1));
            let _ = port.flush_buffers();
            port.write(&packet).expect("write");
            let started = std::time::Instant::now();
            let mut all = Vec::new();
            let mut buf = [0u8; 2048];
            while started.elapsed() < Duration::from_millis(50) {
                if let Ok(n) = port.read(&mut buf) {
                    if n > 0 {
                        all.extend_from_slice(&buf[..n]);
                    }
                }
            }
            println!("{} bytes back for {} ids x {} bytes", all.len(), ids.len(), len);
            // Walk the stream packet by packet.
            let mut i = 0;
            let mut good = 0;
            let mut bad = 0;
            let mut seen = Vec::new();
            while i + 10 <= all.len() {
                if all[i..i + 4] == [0xFF, 0xFF, 0xFD, 0x00] {
                    let length = u16::from_le_bytes([all[i + 5], all[i + 6]]) as usize;
                    let total = 7 + length;
                    if i + total <= all.len() {
                        let pkt = &all[i..i + total];
                        let crc = u16::from_le_bytes([pkt[total - 2], pkt[total - 1]]);
                        if crc == drlabs_dynamixel::protocol::update_crc(0, &pkt[..total - 2]) {
                            good += 1;
                            seen.push(pkt[4]);
                            i += total;
                            continue;
                        }
                        bad += 1;
                        println!("  bad crc at {i}: {:02X?}", pkt);
                    } else {
                        println!("  truncated packet at {i}: {:02X?}", &all[i..]);
                        break;
                    }
                }
                i += 1;
            }
            println!("  {good} valid packets from ids {seen:?}, {bad} crc failures");
            if all.len() < 400 {
                println!("  raw: {:02X?}", all);
            }
        }
        // rdt <ids> <counts>: set Return Delay Time (reg 9, EEPROM) on every id.
        "rdt" => {
            let counts: u8 = args.get(5).and_then(|s| s.parse().ok()).unwrap_or(0);
            for id in parse_ids(&args[4]) {
                let _ = driver.write_u8(id, 64, 0);
                let r = driver.write_u8(id, 9, counts);
                let back = driver.read(id, 9, 1).map(|b| b[0]);
                println!("id {id}: rdt={counts} {r:?} readback={back:?}");
            }
            std::thread::sleep(Duration::from_millis(200));
        }
        other => eprintln!("unknown mode {other}"),
    }
}
