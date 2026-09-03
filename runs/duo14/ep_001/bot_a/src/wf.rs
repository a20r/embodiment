use std::fs::{File, OpenOptions};
use std::io::{Read, Write, BufWriter};
use std::time::{Duration, Instant};

fn rp(p: &str) -> String {
    let mut s = String::new();
    if let Ok(mut f) = File::open(format!("/dev/robot/{}", p)) {
        let _ = f.read_to_string(&mut s);
    }
    return s.trim().to_string();
}
fn wp(p: &str, v: f64) {
    if let Ok(mut f) = File::create(format!("/dev/robot/{}", p)) {
        let _ = f.write_all(format!("{}\n", v).as_bytes());
    }
}
fn pf(s: &str) -> f64 { s.parse().unwrap_or(-1.0) }

fn main() {
    let mut log = BufWriter::new(OpenOptions::new().create(true).append(true).open("/tmp/wf.log").unwrap());
    let t0 = Instant::now();
    let mut last_tx = Instant::now();
    let mut lb = [0.30f64; 16];
    let mut last_er: i64 = pf(&rp("d6")) as i64;
    let mut last_el: i64 = pf(&rp("d9")) as i64;
    let mut stuck_ref = (last_er, last_el, Instant::now());
    let mut odo = (last_er, last_el, 0.0f64, 0.0f64); // er, el, x, y
    let mut goal_seen = 0u32;

    loop {
        // --- read + smooth beams ---
        let mut b = [0.30f64; 16];
        for _ in 0..2 {
            let raw: Vec<f64> = rp("d2").split(',').map(pf).collect();
            if raw.len() == 16 {
                for k in 0..16 {
                    if raw[k] >= 0.0 {
                        b[k] = (raw[k] + lb[k]) / 2.0;
                        lb[k] = raw[k];
                    } else { b[k] = lb[k]; }
                }
            }
            std::thread::sleep(Duration::from_millis(40));
        }
        let mut heading = pf(&rp("d4"));
        let er_raw = pf(&rp("d6")) as i64;
        let el_raw = pf(&rp("d9")) as i64;
        if er_raw >= 0 { last_er = er_raw; }
        if el_raw >= 0 { last_el = el_raw; }
        let er = last_er; let el = last_el;
        let d3 = rp("d3");
        let d11 = pf(&rp("d11"));

        // --- flags ---
        if d3.contains("goal=1") || d3.contains("here=1") {
            goal_seen += 1;
            wp("d1", 0.0); wp("d7", 0.0);
            let _ = writeln!(log, "t={} !!!FLAGS {} d11={:.2} hdg={:.1} enc={};{}", t0.elapsed().as_secs(), d3, d11, heading, er, el);
            let _ = log.flush();
            if goal_seen > 3 {
                if last_tx.elapsed() > Duration::from_secs(2) {
                    last_tx = Instant::now();
                    if let Ok(mut f) = File::create("/dev/robot/d8") {
                        let _ = f.write_all(format!("R1 AT-GOAL hdg={:.0} enc={};{}\n", heading, er, el).as_bytes());
                    }
                }
                std::thread::sleep(Duration::from_millis(400));
            }
            continue;
        }

        // --- odometry ---
        {
            let (pr, pl, x, y) = odo;
            let hrad = heading * std::f64::consts::PI / 180.0;
            let adv = ((er - pr) + (el - pl)) as f64 / 2.0;
            odo = (er, el, x + hrad.cos() * adv, y + hrad.sin() * adv);
        }

        let f = b[15].min(b[0]).min(b[1]);
        let f5 = f.min(b[14]).min(b[2]);
        let ld = b[12].min(b[13]);        // left side wall dist
        let rd = b[3].min(b[4]);          // right side wall dist
        let left_open = b[11] + b[12] + b[13] + b[14];
        let right_open = b[2] + b[3] + b[4] + b[5];

        // --- stuck check ---
        {
            let (sr, sl, st) = stuck_ref;
            if st.elapsed() > Duration::from_secs(2) {
                let mv = (er - sr).abs() + (el - sl).abs();
                stuck_ref = (er, el, Instant::now());
                if mv < 12 {
                    let _ = writeln!(log, "t={} STUCK mv={} f={:.2} ld={:.2}", t0.elapsed().as_secs(), mv, f, ld);
                    let _ = log.flush();
                    let end = Instant::now() + Duration::from_millis(1300);
                    while Instant::now() < end { wp("d1", -1.8); wp("d7", -1.8); std::thread::sleep(Duration::from_millis(40)); }
                    wp("d1", 0.0); wp("d7", 0.0);
                    let dir = if right_open > left_open { 1.0 } else { -1.0 };
                    let end = Instant::now() + Duration::from_millis(1500);
                    while Instant::now() < end { wp("d1", 5.0 * dir); wp("d7", -5.0 * dir); std::thread::sleep(Duration::from_millis(40)); }
                    wp("d1", 0.0); wp("d7", 0.0);
                    last_er = pf(&rp("d6")) as i64; last_el = pf(&rp("d9")) as i64;
                    if last_er >= 0 && last_el >= 0 { stuck_ref = (last_er, last_el, Instant::now()); odo = (last_er, last_el, odo.2, odo.3); }
                    continue;
                }
            }
        }

        let mut mode = "go";
        let (mut l, mut r) = (0.0f64, 0.0f64);
        if f5 < 0.45 {
            // wall close ahead: rotate toward more open side
            mode = "turnT";
            let dir = if b[13] + b[14] + b[15] >= b[1] + b[2] + b[3] { -1.0 } else { 1.0 };
            for _bi in 0..8 {
            let end = Instant::now() + Duration::from_millis(450);
            while Instant::now() < end { wp("d1", 5.0 * dir); wp("d7", -5.0 * dir); std::thread::sleep(Duration::from_millis(40)); }
            wp("d1", 0.0); wp("d7", 0.0);
            std::thread::sleep(Duration::from_millis(120));
            let raw2: Vec<f64> = rp("d2").split(',').map(pf).collect();
            if raw2.len() == 16 {
                let fnow = raw2[15].min(raw2[0]).min(raw2[1]);
                if fnow > 0.55 { break; }
            }
        }
        } else if f < 0.50 {
            mode = "slowsq";
            // careful approach + slight turn to more open side
            let dir = if b[13] + b[14] + b[15] >= b[1] + b[2] + b[3] { -1.0 } else { 1.0 };
            l = 0.6 + 0.35 * dir; r = 0.6 - 0.35 * dir;
        } else if ld > 0.85 {
            // left opening: turn left (left-hand rule)
            mode = "turnL";
            l = 0.25; r = 1.1;
        } else {
            // follow left wall at 0.30
            mode = "follow";
            let err = ld - 0.30;
            let steer = (err * 2.2).clamp(-0.55, 0.55);
            let v = (f * 1.5).clamp(0.7, 1.4);
            l = v - steer; r = v + steer;
        }
        wp("d1", l); wp("d7", r);

        if last_tx.elapsed() > Duration::from_secs(4) {
            last_tx = Instant::now();
            if let Ok(mut f2) = File::create("/dev/robot/d8") {
                let _ = f2.write_all(format!("R1 xy={:.0};{:.0} hdg={:.0} t={}\n", odo.2, odo.3, heading, t0.elapsed().as_secs()).as_bytes());
            }
        }
        heading = pf(&rp("d4"));
        let _ = writeln!(log, "t={} {} hdg={:.1} enc={};{} xy={:.0};{:.0} f={:.2} f5={:.2} ld={:.2} rd={:.2} l={:.2} r={:.2} {} d11={:.2} B={}",
            t0.elapsed().as_secs(), mode, heading, er, el, odo.2, odo.3, f, f5, ld, rd, l, r, d3, d11,
            b.iter().map(|v| format!("{:.2}", v)).collect::<Vec<_>>().join(","));
        let _ = log.flush();
        std::thread::sleep(Duration::from_millis(70));
    }
}
