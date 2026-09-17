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
    let mut log = BufWriter::new(OpenOptions::new().create(true).append(true).open("/tmp/nav2.log").unwrap());
    let t0 = Instant::now();
    let mut last_tx = Instant::now();
    let mut last_enc = (0i64, 0i64); let mut last_move = Instant::now();
    let mut stuck_count = 0u32;
    let mut mode = String::from("start");
    let mut recover_until = Instant::now();
    let mut recover = |l: f64, r: f64, ms: u64, log: &mut BufWriter<File>, t0: &Instant, mode: &str| {
        let end = Instant::now() + Duration::from_millis(ms);
        while Instant::now() < end {
            wp("d1", l); wp("d7", r);
            std::thread::sleep(Duration::from_millis(60));
        }
        let _ = writeln!(log, "t={} RECOVER {} l={:.2} r={:.2}", t0.elapsed().as_secs(), mode, l, r);
    };

    loop {
        let d2raw = rp("d2");
        let beams: Vec<f64> = d2raw.split(',').map(pf).collect();
        if beams.len() < 16 { continue; }
        let heading = pf(&rp("d4"));
        let enc_r = pf(&rp("d6")) as i64; let enc_l = pf(&rp("d9")) as i64;
        let d3 = rp("d3");
        let d11 = pf(&rp("d11"));

        // stuck detection: commanded move but encoders frozen
        let moved = (enc_r - last_enc.0).abs() + (enc_l - last_enc.1).abs();
        last_enc = (enc_r, enc_l);
        if moved < 2 { stuck_count += 1; } else { stuck_count = 0; }
        if stuck_count > 12 && Instant::now() > recover_until {
            // wedged: back up and turn right (arbitrary), 2s
            stuck_count = 0;
            recover_until = Instant::now() + Duration::from_secs(3);
            recover(-1.5, -1.5, 1200, &mut log, &t0, "back");
            recover(-1.2, 1.2, 900, &mut log, &t0, "turn");
            last_enc = (pf(&rp("d6")) as i64, pf(&rp("d9")) as i64);
            continue;
        }

        // find gaps: contiguous runs of beams with range > 0.45
        let open = |k: usize| beams[k % 16] > 0.45;
        let mut best_err = 0.0f64; let mut best_score = -1.0f64; let mut best_r = 0.0f64;
        let mut k = 0usize;
        while k < 16 {
            if open(k) {
                let mut j = k; let mut sum = 0.0; let mut wsum = 0.0; let mut cnt = 0;
                while j < 16 && open(j) {
                    let ang = (j as f64) * 22.5; // deg cw from forward
                    let w = beams[j];
                    sum += ang * w; wsum += w; cnt += 1;
                    j += 1;
                }
                let center = if wsum > 0.0 { sum / wsum } else { (k as f64) * 22.5 };
                let depth = (k..j).map(|i| beams[i]).fold(f64::INFINITY, f64::min); // gap min depth
                // center angle relative to forward: -180..180
                let err = if center > 180.0 { center - 360.0 } else { center };
                let fw = 1.0 - (err.abs() / 180.0); // prefer forward gaps
                let score = depth * cnt as f64 * (0.3 + fw);
                if score > best_score { best_score = score; best_err = err; best_r = depth; }
                k = j;
            } else { k += 1; }
        }
        // if best gap is behind, allow wrap: also consider wrap-around gap
        // (skip for now; rotate when nothing forward)
        let fclear = beams[15].min(beams[0]).min(beams[1]).min(beams[2]).min(beams[14]);
        let (mut l, mut r) = (0.0f64, 0.0f64);
        if best_score < 0.0 || fclear < 0.28 {
            mode = "rotate".into();
            // rotate toward best_err sign (or any open side)
            let dir = if best_err >= 0.0 { 1.0 } else { -1.0 };
            l = 1.5 * dir; r = -1.5 * dir;
        } else {
            mode = "seek".into();
            let steer = (best_err / 22.5) * 0.30; // beams of error -> motor diff
            let base = (best_r * 0.9).clamp(0.5, 1.6);
            l = (base - steer).clamp(-2.0, 2.0);
            r = (base + steer).clamp(-2.0, 2.0);
        }
        wp("d1", l); wp("d7", r);

        if last_tx.elapsed() > Duration::from_secs(4) {
            last_tx = Instant::now();
            if let Ok(mut f) = File::create("/dev/robot/d8") {
                let _ = f.write_all(format!("R1 pos={} {} hdg={:.0} t={}\n", enc_r, enc_l, heading, t0.elapsed().as_secs()).as_bytes());
            }
        }
        let _ = writeln!(log, "t={} m={} hdg={:.1} enc={};{} l={:.2} r={:.2} err={:.0}@{:.2} sc={:.1} {} d11={:.2} d2={}",
            t0.elapsed().as_secs(), mode, heading, enc_r, enc_l, l, r, best_err, best_r, best_score, d3, d11, d2raw);
        let _ = log.flush();
        std::thread::sleep(Duration::from_millis(100));
    }
}
