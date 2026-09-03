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
    let mut log = BufWriter::new(OpenOptions::new().create(true).append(true).open("/tmp/nav6.log").unwrap());
    let t0 = Instant::now();
    let mut last_tx = Instant::now();
    let mut last_er: i64 = pf(&rp("d6")) as i64;
    let mut last_el: i64 = pf(&rp("d9")) as i64;
    let mut last = (last_er, last_el, Instant::now());
    let mut odo = (last_er, last_el, Instant::now());
    let mut x: f64 = 0.0; let mut y: f64 = 0.0;
    let mut lb = [-1.0f64; 16];
    let mut goal_seen = 0u32;

    loop {
        let mut b = [-1.0f64; 16];
        for _ in 0..2 {
            let raw: Vec<f64> = rp("d2").split(',').map(pf).collect();
            if raw.len() == 16 {
                for k in 0..16 {
                    if raw[k] >= 0.0 { b[k] = if lb[k] > 0.0 { (raw[k] + lb[k]) / 2.0 } else { raw[k] }; lb[k] = raw[k]; }
                    else { b[k] = if lb[k] > 0.0 { lb[k] } else { 0.3 }; }
                }
            }
            std::thread::sleep(Duration::from_millis(40));
        }
        let heading = pf(&rp("d4"));
        let er_raw = pf(&rp("d6")) as i64; let el_raw = pf(&rp("d9")) as i64;
        if er_raw >= 0 { last_er = er_raw; }
        if el_raw >= 0 { last_el = el_raw; }
        let er = last_er; let el = last_el;
        let d3 = rp("d3"); let d11 = pf(&rp("d11"));
        if d3.contains("goal=1") || d3.contains("here=1") {
            goal_seen += 1;
            let _ = writeln!(log, "t={} !!!FLAGS {} d11={:.2} xy={:.0};{:.0} hdg={:.1}", t0.elapsed().as_secs(), d3, d11, x, y, heading);
            let _ = log.flush();
            if goal_seen > 5 {
                wp("d1", 0.0); wp("d7", 0.0);
                if last_tx.elapsed() > Duration::from_secs(3) {
                    last_tx = Instant::now();
                    if let Ok(mut f) = File::create("/dev/robot/d8") {
                        let _ = f.write_all(format!("R1 AT GOAL xy={:.0};{:.0} hdg={:.0}\n", x, y, heading).as_bytes());
                    }
                }
                std::thread::sleep(Duration::from_millis(400));
            }
            continue;
        }

        // dead reckon (separate tracker from stuck detection!)
        {
            let (pr, pl, pt) = odo;
            if pt.elapsed() > Duration::from_millis(150) {
                let hrad = heading * std::f64::consts::PI / 180.0;
                let adv = ((er - pr) + (el - pl)) as f64 / 2.0;
                x += hrad.cos() * adv; y += hrad.sin() * adv;
                odo = (er, el, Instant::now());
            }
        }

        // potential field: attract to open beams, repel from close ones
        let mut fx = 0.0f64; let mut fy = 0.0f64;
        for k in 0..16 {
            let ang = (k as f64) * 22.5f64 * std::f64::consts::PI / 180.0; // cw from forward
            let (ux, uy) = (ang.cos(), ang.sin()); // in cw-from-forward frame
            let r = b[k];
            if r > 0.45 {
                let w = r.min(2.0) * r.min(2.0); // attract stronger to far
                fx += w * ux; fy += w * uy;
            }
            if r < 0.55 {
                let w = (0.55 - r).max(0.0) * (0.55 - r).max(0.0) * 6.0; // repel
                fx -= w * ux; fy -= w * uy;
            }
        }
        // desired angle in cw-from-forward frame
        let mut want = fy.atan2(fx) * 180.0 / std::f64::consts::PI; // atan2(y,x): y=sin(cw angle) => positive = right
        // scale to [-180,180]
        if want > 180.0 { want -= 360.0; }
        if want < -180.0 { want += 360.0; }
        let strength = (fx * fx + fy * fy).sqrt();

        let front = b[15].min(b[0]).min(b[1]);
        let near = b.iter().cloned().fold(f64::INFINITY, f64::min);

        let (mut l, mut r) = (0.0f64, 0.0f64);
        let mut mode = "field";
        // stuck?
        {
            let (pr, pl, pt) = last;
            if pt.elapsed() > Duration::from_secs(2) {
                let mv = (er - pr).abs() + (el - pl).abs();
                if mv < 12 && strength > 0.05 {
                    mode = "stuck";
                    let _ = writeln!(log, "t={} STUCK mv={} back+turn", t0.elapsed().as_secs(), mv);
                    let _ = log.flush();
                    let end = Instant::now() + Duration::from_millis(900);
                    while Instant::now() < end { wp("d1", -1.6); wp("d7", -1.6); std::thread::sleep(Duration::from_millis(50)); }
                    let dir = if b[3] + b[4] + b[5] > b[11] + b[12] + b[13] { 1.0 } else { -1.0 };
                    let end = Instant::now() + Duration::from_millis(1300);
                    while Instant::now() < end { wp("d1", 5.0 * dir); wp("d7", -5.0 * dir); std::thread::sleep(Duration::from_millis(50)); }
                    wp("d1", 0.0); wp("d7", 0.0);
                    last = (pf(&rp("d6")) as i64, pf(&rp("d9")) as i64, Instant::now());
                    continue;
                }
                last = (er, el, Instant::now());
            }
        }
        if strength < 0.05 {
            mode = "cage";
            l = 2.0; r = -2.0; // rotate cw hoping to find opening
        } else {
            let steer = (want / 22.5) * 0.30; // deg error -> motor diff
            let v = ((b[0] * 0.7 + b[1] * 0.3) * 1.3).clamp(0.7, 1.7);
            l = (v + steer).clamp(-1.8, 2.2);
            r = (v - steer).clamp(-1.8, 2.2);
            mode = "field";
        }
        wp("d1", l); wp("d7", r);

        if last_tx.elapsed() > Duration::from_secs(4) {
            last_tx = Instant::now();
            if let Ok(mut f) = File::create("/dev/robot/d8") {
                let _ = f.write_all(format!("R1 xy={:.0};{:.0} hdg={:.0} t={}\n", x, y, heading, t0.elapsed().as_secs()).as_bytes());
            }
        }
        let _ = writeln!(log, "t={} {} hdg={:.1} enc={};{} xy={:.0};{:.0} want={:.0} v-str={:.2} l={:.2} r={:.2} {} d11={:.2} b0={:.2} b2={:.2} b13={:.2}",
            t0.elapsed().as_secs(), mode, heading, er, el, x, y, want, strength, l, r, d3, d11, b[0], b[2], b[13]);
        let _ = log.flush();
        std::thread::sleep(Duration::from_millis(80));
    }
}
