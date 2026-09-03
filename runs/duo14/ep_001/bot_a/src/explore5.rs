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

struct Ctl { l: f64, r: f64 }

fn main() {
    let mut log = BufWriter::new(OpenOptions::new().create(true).append(true).open("/tmp/nav5.log").unwrap());
    let t0 = Instant::now();
    let mut last_tx = Instant::now();
    let mut last = (pf(&rp("d6")) as i64, pf(&rp("d9")) as i64, Instant::now());
    let mut x: f64 = 0.0; let mut y: f64 = 0.0;
    let mut lb = [-1.0f64; 16]; // last valid beams
    let mut goal_seen = 0u32;

    loop {
        // read beams (2 samples averaged)
        let mut b = [-1.0f64; 16];
        for _ in 0..2 {
            let raw: Vec<f64> = rp("d2").split(',').map(pf).collect();
            if raw.len() == 16 {
                for k in 0..16 { if raw[k] >= 0.0 { b[k] = if lb[k] > 0.0 { (raw[k] + lb[k]) / 2.0 } else { raw[k] }; lb[k] = raw[k]; } else { b[k] = if lb[k] > 0.0 { lb[k] } else { 0.25 }; } }
            }
            std::thread::sleep(Duration::from_millis(40));
        }
        let heading = pf(&rp("d4"));
        let er = pf(&rp("d6")) as i64; let el = pf(&rp("d9")) as i64;
        let d3 = rp("d3"); let d11 = pf(&rp("d11"));
        if d3.contains("goal=1") || d3.contains("here=1") {
            goal_seen += 1;
            let _ = writeln!(log, "t={} !!!FLAGS {} d11={:.2} xy={:.0};{:.0} hdg={:.1}", t0.elapsed().as_secs(), d3, d11, x, y, heading);
            let _ = log.flush();
            if goal_seen > 5 {
                wp("d1", 0.0); wp("d7", 0.0);
                // stay put; keep beaconing
                if last_tx.elapsed() > Duration::from_secs(3) {
                    last_tx = Instant::now();
                    if let Ok(mut f) = File::create("/dev/robot/d8") {
                        let _ = f.write_all(format!("R1 AT GOAL xy={:.0};{:.0} hdg={:.0}\n", x, y, heading).as_bytes());
                    }
                }
                std::thread::sleep(Duration::from_millis(500));
                continue;
            }
        }

        // dead reckon
        {
            let (pr, pl, pt) = last;
            if pt.elapsed() > Duration::from_millis(150) {
                let hrad = heading * std::f64::consts::PI / 180.0;
                let adv = ((er - pr) + (el - pl)) as f64 / 2.0;
                x += hrad.cos() * adv; y += hrad.sin() * adv;
                last = (er, el, Instant::now());
            }
        }

        // gap analysis
        let mut best_c = 0.0f64; let mut best_score = -1.0f64; let mut best_d = 0.0f64; let mut best_w = 0usize;
        let mut k = 0usize;
        while k < 16 {
            if b[k] > 0.42 {
                let mut j = k; let mut sum = 0.0; let mut wsum = 0.0; let mut cnt = 0; let mut depth = f64::INFINITY;
                while j < 16 && b[j] > 0.42 {
                    sum += (j as f64) * 22.5 * b[j]; wsum += b[j];
                    depth = depth.min(b[j]); cnt += 1; j += 1;
                }
                let mut c = if wsum > 0.0 { sum / wsum } else { k as f64 * 22.5 };
                if c > 180.0 { c -= 360.0; }
                let fw = 1.0 - c.abs() / 220.0;
                let score = depth * (cnt as f64).powf(0.7) * (0.4 + fw);
                if score > best_score { best_score = score; best_c = c; best_d = depth; best_w = cnt; }
                k = j;
            } else { k += 1; }
        }
        let f3 = b[15].min(b[0]).min(b[1]);
        let f5 = f3.min(b[2]).min(b[14]);

        // STUCK check
        let (pr, pl, pt) = last;
        let stuck = pt.elapsed() > Duration::from_secs(2) && ((er - pr).abs() + (el - pl).abs()) < 8;

        let mut ctl = Ctl { l: 0.0, r: 0.0 };
        let mut mode = "idle";
        if stuck {
            mode = "stuck";
            let _ = writeln!(log, "t={} STUCK back+turn hdg={:.1} xy={:.0};{:.0}", t0.elapsed().as_secs(), heading, x, y);
            let _ = log.flush();
            let end = Instant::now() + Duration::from_millis(900);
            while Instant::now() < end { wp("d1", -1.6); wp("d7", -1.6); std::thread::sleep(Duration::from_millis(50)); }
            let dir = if b[3] + b[4] + b[5] > b[11] + b[12] + b[13] { 1.0 } else { -1.0 };
            let end = Instant::now() + Duration::from_millis(1300);
            while Instant::now() < end { wp("d1", 5.0 * dir); wp("d7", -5.0 * dir); std::thread::sleep(Duration::from_millis(50)); }
            wp("d1", 0.0); wp("d7", 0.0);
            last = (pf(&rp("d6")) as i64, pf(&rp("d9")) as i64, Instant::now());
            continue;
        } else if best_score < 0.0 {
            mode = "spin";
            ctl = Ctl { l: 2.0, r: -2.0 };
        } else if best_d > 0.9 && best_c.abs() < 30.0 && f3 > 0.30 {
            // through-slit or open corridor: go, gently centered on gap
            mode = "go";
            let steer = (best_c / 22.5) * 0.25;
            let v = (best_d * 0.8).clamp(0.6, 1.6);
            ctl = Ctl { l: (v + steer).clamp(-1.0, 2.4), r: (v - steer).clamp(-1.0, 2.4) };
        } else if f5 < 0.30 {
            // wall close ahead: back off a touch and re-aim
            mode = "back";
            ctl = Ctl { l: -0.9, r: -0.9 };
            if b[2] > b[14] { ctl = Ctl { l: -0.5, r: -1.3 }; } else { ctl = Ctl { l: -1.3, r: -0.5 }; }
        } else {
            // aim toward best gap
            mode = "aim";
            let dir = if best_c > 0.0 { 1.0 } else { -1.0 };
            let errb = (best_c / 22.5).abs();
            if errb < 1.2 {
                // small error: gentle curve forward
                let steer = (best_c / 22.5) * 0.30;
                let v = (best_d * 0.7).clamp(0.5, 1.2);
                ctl = Ctl { l: (v + steer).clamp(-1.5, 2.0), r: (v - steer).clamp(-1.5, 2.0) };
            } else {
                // rotate in place toward gap
                let ms = (errb * 1900.0) as u64;
                let _ = writeln!(log, "t={} AIMROT err={:.0} d={:.2} ms={} hdg={:.1}", t0.elapsed().as_secs(), best_c, best_d, ms, heading);
                let end = Instant::now() + Duration::from_millis(ms.max(300));
                while Instant::now() < end { wp("d1", 6.0 * dir); wp("d7", -6.0 * dir); std::thread::sleep(Duration::from_millis(50)); }
                wp("d1", 0.0); wp("d7", 0.0);
                last = (pf(&rp("d6")) as i64, pf(&rp("d9")) as i64, Instant::now());
                continue;
            }
        }
        wp("d1", ctl.l); wp("d7", ctl.r);

        if last_tx.elapsed() > Duration::from_secs(4) {
            last_tx = Instant::now();
            if let Ok(mut f) = File::create("/dev/robot/d8") {
                let _ = f.write_all(format!("R1 xy={:.0};{:.0} hdg={:.0} t={}\n", x, y, heading, t0.elapsed().as_secs()).as_bytes());
            }
        }
        let _ = writeln!(log, "t={} {} hdg={:.1} enc={};{} xy={:.0};{:.0} c={:.2}@{:.2} w={} f3={:.2} l={:.2} r={:.2} {} d11={:.2}",
            t0.elapsed().as_secs(), mode, heading, er, el, x, y, best_c, best_d, best_w, f3, ctl.l, ctl.r, d3, d11);
        let _ = log.flush();
        std::thread::sleep(Duration::from_millis(80));
    }
}
