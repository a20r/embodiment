use std::fs::{File, OpenOptions};
use std::io::{Read, Write, BufWriter};
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
fn wp(p: &str, v: f64) {
    if let Ok(mut f) = OpenOptions::new().write(true).custom_flags(2048).open(format!("/dev/robot/{}", p)) {
        let _ = f.write_all(format!("{}\n", v).as_bytes());
    }
}
fn pf(s: &str) -> f64 { s.parse().unwrap_or(-1.0) }

fn avg_d11(n: u32) -> f64 {
    let mut s = 0.0; let mut c = 0;
    for _ in 0..n {
        let v = pf(&rp("d11"));
        if v >= 0.0 { s += v; c += 1; }
        std::thread::sleep(Duration::from_millis(80));
    }
    if c > 0 { s / c as f64 } else { -1.0 }
}

fn main() {
    let mut log = BufWriter::new(OpenOptions::new().create(true).append(true).open("/tmp/climb.log").unwrap());
    let t0 = Instant::now();
    let mut dir: f64 = 0.0; // 0 = straight, accumulate turn bias
    let mut best: f64 = -1.0;
    loop {
        let d0 = avg_d11(12);
        if d0 > best { best = d0; }
        // drive forward 2.5s
        let end = Instant::now() + Duration::from_millis(3000);
        while Instant::now() < end { wp("d1", 1.1); wp("d7", 1.1); std::thread::sleep(Duration::from_millis(50)); }
        wp("d1", 0.0); wp("d7", 0.0);
        std::thread::sleep(Duration::from_millis(300));
        let d1 = avg_d11(12);
        let delta = d1 - d0;
        let mut action = "keep";
        if delta < -0.006 {
            action = "turn60";
            let end = Instant::now() + Duration::from_millis(1700);
            while Instant::now() < end { wp("d1", 5.0); wp("d7", -5.0); std::thread::sleep(Duration::from_millis(40)); }
            wp("d1", 0.0); wp("d7", 0.0);
        } else if delta < -0.001 {
            action = "turn20";
            let end = Instant::now() + Duration::from_millis(600);
            while Instant::now() < end { wp("d1", 5.0); wp("d7", -5.0); std::thread::sleep(Duration::from_millis(40)); }
            wp("d1", 0.0); wp("d7", 0.0);
        }
        let d3 = rp("d3");
        let _ = writeln!(log, "t={} d0={:.3} d1={:.3} delta={:+.3} best={:.3} {} {}",
            t0.elapsed().as_secs(), d0, d1, delta, best, action, d3);
        let _ = log.flush();
        if d3.contains("here=1") { wp("d1", 0.0); wp("d7", 0.0); let _ = writeln!(log, "t={} !!!HERE=1 HOLDING", t0.elapsed().as_secs()); let _ = log.flush(); loop { std::thread::sleep(Duration::from_secs(1)); } }
        std::thread::sleep(Duration::from_millis(200));
    }
}
