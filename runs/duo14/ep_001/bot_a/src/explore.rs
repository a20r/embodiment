use std::fs::{File, OpenOptions};
use std::io::{Read, Write, BufWriter};
use std::time::{Duration, Instant};

fn rp(p: &str) -> String {
    let mut s = String::new();
    if let Ok(mut f) = File::open(format!("/dev/robot/{}", p)) { let _ = f.read_to_string(&mut s); }
    s.trim().to_string()
}
fn wp(p: &str, v: f64) {
    if let Ok(mut f) = File::create(format!("/dev/robot/{}", p)) { let _ = f.write_all(format!("{}\n", v).as_bytes()); }
}
fn pf(s: &str) -> f64 { s.parse().unwrap_or(-1.0) }

fn main() {
    let mut log = BufWriter::new(OpenOptions::new().create(true).append(true).open("/tmp/nav.log").unwrap());
    let t0 = Instant::now();
    let mut last_tx = Instant::now();
    let mut mode = String::from("start");
    loop {
        // read sensors
        let d2raw = rp("d2");
        let beams: Vec<f64> = d2raw.split(',').map(pf).collect();
        if beams.len() < 16 { continue; }
        let heading = pf(&rp("d4"));
        let enc_r = pf(&rp("d6")); let enc_l = pf(&rp("d9"));
        let d3 = rp("d3");
        let d11 = pf(&rp("d11"));

        // beam angles: k*22.5 deg clockwise from forward
        // steer toward weighted max
        let mut best_k = 0usize; let mut best_r = -1.0;
        for k in 0..16 { if beams[k] > best_r { best_r = beams[k]; best_k = k; } }
        // forward-ish clearance
        let fclear = beams[15].min(beams[0]).min(beams[1]).min(beams[2]).min(beams[14]);
        let (mut l, mut r) = (0.0f64, 0.0f64);
        let err = if best_k <= 8 { best_k as f64 } else { best_k as f64 - 16.0 }; // -7..8 beams
        if fclear < 0.30 {
            // too close ahead: rotate toward best opening
            mode = "avoid".into();
            if err > 0.0 { l = 1.2; r = -1.2; } else { l = -1.2; r = 1.2; }
        } else {
            mode = "seek".into();
            // proportional steering toward best beam (22.5deg per beam)
            let steer = err * 0.35; // motor diff
            let base = (fclear * 1.2).clamp(0.4, 1.4);
            l = base - steer; r = base + steer;
            l = l.clamp(-2.5, 2.5); r = r.clamp(-2.5, 2.5);
        }
        wp("d1", l); wp("d7", r);

        // radio beacon every 4s
        if last_tx.elapsed() > Duration::from_secs(4) {
            last_tx = Instant::now();
            if let Ok(mut f) = File::create("/dev/robot/d8") {
                let _ = f.write_all(format!("R1 pos={} {} hdg={:.0} t={}\n", enc_r as i64, enc_l as i64, heading, t0.elapsed().as_secs()).as_bytes());
            }
        }
        let line = format!("t={} m={} hdg={} enc={};{} l={:.2} r={:.2} best={}@{:.2} fc={:.2} {} d11={:.2} d2={}\n",
            t0.elapsed().as_secs(), mode, heading, enc_r as i64, enc_l as i64, l, r, best_k, best_r, fclear, d3, d11, d2raw);
        let _ = log.write_all(line.as_bytes()); let _ = log.flush();
        std::thread::sleep(Duration::from_millis(100));
    }
}
