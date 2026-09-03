use std::fs::{File, OpenOptions};
use std::io::{Read, Write, Seek};
use std::time::{Duration, Instant};
use std::os::unix::fs::OpenOptionsExt;

fn rp(p: &str) -> String {
    let deadline = Instant::now() + Duration::from_millis(120);
    if let Ok(mut f) = OpenOptions::new().read(true).custom_flags(2048).open(format!("/dev/robot/{}", p)) {
        let mut buf = [0u8; 512];
        loop {
            match f.read(&mut buf) {
                Ok(n) if n > 0 => { return String::from_utf8_lossy(&buf[..n]).trim().to_string(); }
                _ => {}
            }
            if Instant::now() > deadline { break; }
            std::thread::sleep(Duration::from_millis(4));
        }
    }
    return String::new();
}
fn wp(p: &str, v: &str) -> bool {
    if let Ok(mut f) = OpenOptions::new().write(true).custom_flags(2048).open(format!("/dev/robot/{}", p)) {
        return f.write_all(format!("{}\n", v).as_bytes()).is_ok();
    }
    return false;
}
fn pf(s: &str) -> f64 { s.parse().unwrap_or(-1.0) }

fn main() {
    let mut last_size: u64 = 0;
    if let Ok(md) = std::fs::metadata("/tmp/inbox.log") { last_size = md.len(); }
    let t0 = Instant::now();
    loop {
        std::thread::sleep(Duration::from_millis(700));
        // check inbox growth
        let sz = std::fs::metadata("/tmp/inbox.log").map(|m| m.len()).unwrap_or(0);
        let mut new_msgs = 0u32;
        if sz > last_size {
            if let Ok(mut f) = File::open("/tmp/inbox.log") {
                let _ = f.seek(std::io::SeekFrom::Start(last_size));
                let mut s = String::new();
                let _ = f.read_to_string(&mut s);
                new_msgs = s.lines().filter(|l| l.contains("PING")).count() as u32;
                // reply once per batch
                if new_msgs > 0 {
                    let er = pf(&rp("d6")) as i64; let el = pf(&rp("d9")) as i64;
                    let hd = pf(&rp("d4"));
                    let dv = pf(&rp("d11"));
                    let _ = wp("d8", &format!("ackA1 R1 d11={:.2} er={} el={} hdg={:.0}", dv, er, el, hd));
                }
            }
            last_size = sz;
        }
        // append correlation log
        let er = pf(&rp("d6")) as i64; let el = pf(&rp("d9")) as i64;
        let hd = pf(&rp("d4"));
        if let Ok(mut lg) = OpenOptions::new().create(true).append(true).open("/tmp/radio_pos.log") {
            let _ = writeln!(lg, "{} {} {} {:.1} {}", t0.elapsed().as_secs(), er, el, hd, new_msgs);
        }
    }
}
