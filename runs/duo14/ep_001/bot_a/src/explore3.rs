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
    let mut log = BufWriter::new(OpenOptions::new().create(true).append(true).open("/tmp/nav3.log").unwrap());
    let t0 = Instant::now();
    let mut last_tx = Instant::now();
    let mut last = (0i64, 0i64, Instant::now());
    let mut stuck = false;
    let mut recov_until = Instant::now();
    let mut smooth: [f64; 16] = [-1.0; 16];
    let mut x: f64 = 0.0; let mut y: f64 = 0.0;
    let mut mode = String::from("start");
    let mut best_err: f64 = 0.0;
    let mut best_score: f64 = -1.0;
    let mut best_depth: f64 = 0.0;

    loop {
        let d2raw = rp("d2");
        let raw: Vec<f64> = d2raw.split(',').map(pf).collect();
        if raw.len() < 16 { continue; }
        for k in 0..16 {
            if raw[k] >= 0.0 { smooth[k] = raw[k]; }
            else {
                let a = smooth[(k + 15) % 16]; let b = smooth[(k + 1) % 16];
                if a > 0.0 && b > 0.0 { smooth[k] = (a + b) / 2.0; }
                else if a > 0.0 { smooth[k] = a; } else if b > 0.0 { smooth[k] = b; }
            }
        }
        let heading = pf(&rp("d4"));
        let er = pf(&rp("d6")) as i64; let el = pf(&rp("d9")) as i64;
        let d3 = rp("d3");
        let d11 = pf(&rp("d11"));

        let (pr, pl, pt) = last;
        if pt.elapsed() > Duration::from_secs(2) {
            let moved = (er - pr).abs() + (el - pl).abs();
            last = (er, el, Instant::now());
            stuck = moved < 8;
        }

        let (mut l, mut r) = (0.0f64, 0.0f64);
        if Instant::now() < recov_until {
            mode = "recover".into();
        } else if stuck {
            let right_room: f64 = smooth[1] + smooth[2] + smooth[3] + smooth[4];
            let left_room: f64 = smooth[12] + smooth[13] + smooth[14] + smooth[15];
            let dir = if right_room > left_room { 1.0 } else { -1.0 };
            let _ = writeln!(log, "t={} STUCK! recov dir={}", t0.elapsed().as_secs(), dir);
            let _ = log.flush();
            let end = Instant::now() + Duration::from_millis(1100);
            while Instant::now() < end { wp("d1", -1.8); wp("d7", -1.8); std::thread::sleep(Duration::from_millis(50)); }
            let end = Instant::now() + Duration::from_millis(1400);
            while Instant::now() < end { wp("d1", 2.0 * dir); wp("d7", -2.0 * dir); std::thread::sleep(Duration::from_millis(50)); }
            recov_until = Instant::now() + Duration::from_millis(400);
            last = (pf(&rp("d6")) as i64, pf(&rp("d9")) as i64, Instant::now());
            continue;
        } else {
            let open = |k: usize| smooth[k % 16] > 0.42;
            best_score = -1.0;
            let mut k = 0usize;
            while k < 16 {
                if open(k) {
                    let mut j = k; let mut sum = 0.0; let mut wsum = 0.0; let mut cnt = 0; let mut depth = f64::INFINITY;
                    while j < 16 && open(j) {
                        let ang = (j as f64) * 22.5;
                        sum += ang * smooth[j]; wsum += smooth[j]; cnt += 1;
                        depth = depth.min(smooth[j]);
                        j += 1;
                    }
                    let mut center = if wsum > 0.0 { sum / wsum } else { k as f64 * 22.5 };
                    if center > 180.0 { center -= 360.0; }
                    let fw = 1.0 - (center.abs() / 200.0);
                    let score = depth * (cnt as f64) * (0.35 + fw);
                    if score > best_score { best_score = score; best_err = center; best_depth = depth; }
                    k = j;
                } else { k += 1; }
            }
            if best_score < 0.0 {
                mode = "rotate".into();
                l = 1.5; r = -1.5;
            } else {
                mode = "seek".into();
                let steer = (best_err / 22.5) * 0.32;
                let base = (best_depth * 0.85).clamp(0.5, 1.5);
                l = (base + steer).clamp(-2.2, 2.2);
                r = (base - steer).clamp(-2.2, 2.2);
            }
        }
        wp("d1", l); wp("d7", r);

        let dr = (er - pr) as f64; let dl = (el - pl) as f64;
        let hrad = heading * std::f64::consts::PI / 180.0;
        x += hrad.cos() * (dr + dl) / 2.0;
        y += hrad.sin() * (dr + dl) / 2.0;

        if last_tx.elapsed() > Duration::from_secs(4) {
            last_tx = Instant::now();
            if let Ok(mut f) = File::create("/dev/robot/d8") {
                let _ = f.write_all(format!("R1 xy={:.0};{:.0} hdg={:.0} t={}\n", x, y, heading, t0.elapsed().as_secs()).as_bytes());
            }
        }
        let _ = writeln!(log, "t={} m={} hdg={:.1} enc={};{} xy={:.0};{:.0} l={:.2} r={:.2} err={:.0}@{:.2} {} d11={:.2} d2={}",
            t0.elapsed().as_secs(), mode, heading, er, el, x, y, l, r, best_err, best_depth, d3, d11, d2raw);
        let _ = log.flush();
        std::thread::sleep(Duration::from_millis(100));
    }
}
