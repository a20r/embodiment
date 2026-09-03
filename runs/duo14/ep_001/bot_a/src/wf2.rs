use std::fs::{File, OpenOptions};
use std::os::unix::fs::OpenOptionsExt;
use std::io::{Read, Write, BufWriter};
use std::time::{Duration, Instant};

fn rp(p: &str) -> String {
    let deadline = Instant::now() + Duration::from_millis(150);
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

fn main() {
    let mut log = BufWriter::new(OpenOptions::new().create(true).append(true).open("/tmp/wf2.log").unwrap());
    let t0 = Instant::now();
    let mut last_tx = Instant::now();
    let mut lb = [0.30f64; 16];
    let mut last_er: i64 = pf(&rp("d6")) as i64;
    let mut last_el: i64 = pf(&rp("d9")) as i64;
    let mut stuck_ref = (last_er, last_el, Instant::now());
    let mut odo = (last_er, last_el, 0.0f64, 0.0f64);
    let mut goal_seen = 0u32;
    let mut consec_stuck = 0u32;

    loop {
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
        let heading = pf(&rp("d4"));
        let er_raw = pf(&rp("d6")) as i64;
        let el_raw = pf(&rp("d9")) as i64;
        if er_raw >= 0 { last_er = er_raw; }
        if el_raw >= 0 { last_el = el_raw; }
        let er = last_er; let el = last_el;
        let d3 = rp("d3");
        let d11 = pf(&rp("d11"));

        if d3.contains("goal=1") || d3.contains("here=1") {
            goal_seen += 1;
            wp("d1", 0.0); wp("d7", 0.0);
            let _ = writeln!(log, "t={} !!!FLAGS {} d11={:.2} hdg={:.1} enc={};{}", t0.elapsed().as_secs(), d3, d11, heading, er, el);
            let _ = log.flush();
            if goal_seen > 3 {
                if last_tx.elapsed() > Duration::from_secs(2) {
                    last_tx = Instant::now();
                    if let Ok(mut f2) = File::create("/dev/robot/d8") {
                        let _ = f2.write_all(format!("R1 AT-GOAL hdg={:.0} enc={};{}\n", heading, er, el).as_bytes());
                    }
                }
                std::thread::sleep(Duration::from_millis(400));
            }
            continue;
        }

        {
            let (pr, pl, x, y) = odo;
            let hrad = heading * std::f64::consts::PI / 180.0;
            let adv = ((er - pr) + (el - pl)) as f64 / 2.0;
            odo = (er, el, x + hrad.cos() * adv, y + hrad.sin() * adv);
        }

        // field: attract open beams (>0.55) weight r; repel only r<0.30 weight 3*(0.30-r)
        let mut fx = 0.0f64; let mut fy = 0.0f64;
        for k in 0..16 {
            let ang = (k as f64) * 22.5f64 * std::f64::consts::PI / 180.0;
            let (ux, uy) = (ang.cos(), ang.sin());
            let r = b[k];
            if r > 0.55 { fx += r * ux; fy += r * uy; }
            if r < 0.30 { let w = (0.30 - r) * 3.0; fx -= w * ux; fy -= w * uy; }
        }
        let strength = (fx * fx + fy * fy).sqrt();
        let mut want = fy.atan2(fx) * 180.0 / std::f64::consts::PI;
        if want > 180.0 { want -= 360.0; }
        if want < -180.0 { want += 360.0; }

        // stuck
        {
            let (sr, sl, st) = stuck_ref;
            if st.elapsed() > Duration::from_secs(2) {
                let mv = (er - sr).abs() + (el - sl).abs();
                stuck_ref = (er, el, Instant::now());
                if mv < 12 {
                    consec_stuck += 1;                    let _ = writeln!(log, "t={} STUCK mv={} want={:.0}", t0.elapsed().as_secs(), mv, want);
                    let _ = log.flush();
                    let end = Instant::now() + Duration::from_millis(1300);
                    while Instant::now() < end { wp("d1", -1.8); wp("d7", -1.8); std::thread::sleep(Duration::from_millis(40)); }
                    wp("d1", 0.0); wp("d7", 0.0);
                    let dir = if b[1] + b[2] + b[3] >= b[13] + b[14] + b[15] { 1.0 } else { -1.0 };
                    let turn_ms = if consec_stuck >= 2 { 3500 } else { 1600 };
                    let end = Instant::now() + Duration::from_millis(turn_ms);
                    while Instant::now() < end { wp("d1", 5.0 * dir); wp("d7", -5.0 * dir); std::thread::sleep(Duration::from_millis(40)); }
                    wp("d1", 0.0); wp("d7", 0.0);
                    consec_stuck = 0; let nr = pf(&rp("d6")) as i64; let nl = pf(&rp("d9")) as i64;
                    if nr >= 0 && nl >= 0 { last_er = nr; last_el = nl; stuck_ref = (nr, nl, Instant::now()); odo = (nr, nl, odo.2, odo.3); }
                    continue;
                }
            }
        }

        let (mut l, mut r) = (0.0f64, 0.0f64);
        let mut mode = "field";
        if strength < 0.08 {
            mode = "cage";
            l = 2.0; r = -2.0;
        } else {
            let steer = (want / 22.5) * 0.30;
            let v = (0.6 + 0.7 * b[0]).clamp(0.7, 1.8);
            l = (v + steer).clamp(-1.6, 2.0);
            r = (v - steer).clamp(-1.6, 2.0);
        }
        wp("d1", l); wp("d7", r);

        if last_tx.elapsed() > Duration::from_secs(2) {
            last_tx = Instant::now();
            if let Ok(mut f2) = File::create("/dev/robot/d8") {
                let _ = f2.write_all(format!("R1 xy={:.0};{:.0} hdg={:.0} t={}\n", odo.2, odo.3, heading, t0.elapsed().as_secs()).as_bytes());
            }
        }
        let _ = writeln!(log, "t={} {} hdg={:.1} enc={};{} xy={:.0};{:.0} want={:.0} str={:.2} l={:.2} r={:.2} {} d11={:.2} B={}",
            t0.elapsed().as_secs(), mode, heading, er, el, odo.2, odo.3, want, strength, l, r, d3, d11,
            b.iter().map(|v| format!("{:.2}", v)).collect::<Vec<_>>().join(","));
        let _ = log.flush();
        std::thread::sleep(Duration::from_millis(70));
    }
}
