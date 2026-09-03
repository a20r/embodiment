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

fn spin(dir: f64, v: f64, ms: u64) {
    let end = Instant::now() + Duration::from_millis(ms);
    while Instant::now() < end { wp("d1", v * dir); wp("d7", -v * dir); std::thread::sleep(Duration::from_millis(50)); }
    wp("d1", 0.0); wp("d7", 0.0);
}
fn drive(v: f64, ms: u64) {
    let end = Instant::now() + Duration::from_millis(ms);
    while Instant::now() < end { wp("d1", v); wp("d7", v); std::thread::sleep(Duration::from_millis(50)); }
    wp("d1", 0.0); wp("d7", 0.0);
}

fn main() {
    let mut log = BufWriter::new(OpenOptions::new().create(true).append(true).open("/tmp/nav4.log").unwrap());
    let t0 = Instant::now();
    let mut last_tx = Instant::now();
    let mut smooth: [f64; 16] = [-1.0; 16];
    let mut last_beams: [f64; 16] = [-1.0; 16];
    let mut x: f64 = 0.0; let mut y: f64 = 0.0;
    let mut last = (pf(&rp("d6")) as i64, pf(&rp("d9")) as i64, Instant::now());
    let mut goal_seen = false;

    loop {
        // refresh smoothed beams
        for _ in 0..3 {
            let d2raw = rp("d2");
            let raw: Vec<f64> = d2raw.split(',').map(pf).collect();
            if raw.len() < 16 { continue; }
            for k in 0..16 {
                if raw[k] >= 0.0 {
                    smooth[k] = if last_beams[k] > 0.0 { (raw[k] + last_beams[k]) / 2.0 } else { raw[k] };
                    last_beams[k] = raw[k];
                }
            }
            std::thread::sleep(Duration::from_millis(30));
        }
        let heading = pf(&rp("d4"));
        let er = pf(&rp("d6")) as i64; let el = pf(&rp("d9")) as i64;
        let d3 = rp("d3"); let d11 = pf(&rp("d11"));
        if d3.contains("goal=1") || d3.contains("here=1") { goal_seen = true; }

        // dead reckon
        let (pr, pl, pt) = last;
        if pt.elapsed() > Duration::from_millis(200) {
            let hrad = heading * std::f64::consts::PI / 180.0;
            let adv = ((er - pr) + (el - pl)) as f64 / 2.0;
            x += hrad.cos() * adv; y += hrad.sin() * adv;
            last = (er, el, Instant::now());
        }

        // PLAN: find best gap across all beams (width-weighted, prefer forward)
        let mut best_center = 0.0f64; let mut best_score = -1.0f64; let mut best_depth = 0.0f64;
        let mut k = 0usize;
        while k < 16 {
            if smooth[k] > 0.45 {
                let mut j = k; let mut sum = 0.0; let mut wsum = 0.0; let mut cnt = 0; let mut depth = f64::INFINITY;
                while j < 16 && smooth[j] > 0.45 {
                    sum += (j as f64) * 22.5 * smooth[j]; wsum += smooth[j];
                    depth = depth.min(smooth[j]); cnt += 1; j += 1;
                }
                let mut c = if wsum > 0.0 { sum / wsum } else { k as f64 * 22.5 };
                if c > 180.0 { c -= 360.0; }
                let fw = 1.0 - c.abs() / 220.0;
                let score = depth * (cnt as f64).powf(0.7) * (0.4 + fw);
                if score > best_score { best_score = score; best_center = c; best_depth = depth; }
                k = j;
            } else { k += 1; }
        }

        // front clearance (beams 15,0,1,2,14)
        let fmin = smooth[15].min(smooth[0]).min(smooth[1]).min(smooth[2]).min(smooth[14]);

        if best_score < 0.0 {
            let _ = writeln!(log, "t={} NOTHING-OPEN spin", t0.elapsed().as_secs());
            spin(1.0, 2.0, 1500);
        } else if fmin < 0.38 || best_center.abs() > 40.0 {
            // AIM: rotate toward best_center
            let dir = if best_center > 0.0 { 1.0 } else { -1.0 };
            let err_beams = (best_center / 22.5).abs();
            let ms = (err_beams * 1900.0) as u64; // ~1 beam/1.9s at v=6
            let _ = writeln!(log, "t={} AIM err={:.0} depth={:.2} ms={} hdg={:.1} xy={:.0};{:.0}", t0.elapsed().as_secs(), best_center, best_depth, ms, heading, x, y);
            spin(dir, 6.0, ms.max(150));
        } else {
            // GO: drive forward, aborting if wall appears
            let v = (fmin * 1.6).clamp(0.8, 2.2);
            let ms = 700u64;
            let _ = writeln!(log, "t={} GO v={:.2} fmin={:.2} hdg={:.1} xy={:.0};{:.0} {}", t0.elapsed().as_secs(), v, fmin, heading, x, y, d3);
            drive(v, ms);
        }

        if last_tx.elapsed() > Duration::from_secs(4) {
            last_tx = Instant::now();
            if let Ok(mut f) = File::create("/dev/robot/d8") {
                let _ = f.write_all(format!("R1 xy={:.0};{:.0} hdg={:.0} t={} goal={}\n", x, y, heading, t0.elapsed().as_secs(), goal_seen).as_bytes());
            }
        }
        let _ = log.flush();
        if goal_seen {
            let _ = writeln!(log, "t={} !!! GOAL FLAG SET: {} d11={:.2}", t0.elapsed().as_secs(), d3, d11);
            let _ = log.flush();
            // stop and hold
            wp("d1", 0.0); wp("d7", 0.0);
        }
        std::thread::sleep(Duration::from_millis(60));
    }
}
