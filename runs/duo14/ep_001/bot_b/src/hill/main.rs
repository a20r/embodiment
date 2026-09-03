use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::thread;
use std::time::{Duration, Instant};

fn set(port: &str, v: &str) { if let Ok(mut f) = File::create(format!("/dev/robot/{}", port)) { let _ = writeln!(f, "{}", v); } }
fn drive(l: f64, r: f64) { set("d1", &format!("{}", l)); set("d7", &format!("{}", r)); }
fn radio(msg: &str) { if let Ok(mut f) = File::create("/dev/robot/d8") { let _ = writeln!(f, "{}", msg); } }
fn evlog(tag: &str, msg: &str) {
    if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open("/tmp/hill.log") {
        let _ = writeln!(f, "{:.2} {} {}", Instant::now().elapsed().as_secs_f64(), tag, msg);
    }
}
fn read_str(port: &str) -> Option<String> {
    if let Ok(f) = File::open(format!("/dev/robot/{}", port)) {
        let mut r = BufReader::new(f);
        let mut s = String::new();
        if r.read_line(&mut s).is_ok() && !s.trim().is_empty() { return Some(s.trim().to_string()); }
    }
    None
}
fn heading() -> f64 { read_str("d4").and_then(|s| s.trim().parse::<f64>().ok()).unwrap_or(f64::NAN) }
fn norm(a: f64) -> f64 { let mut x = a % 360.0; if x < 0.0 { x += 360.0; } x }
fn angdiff(a: f64, b: f64) -> f64 { let d = norm(a - b); if d > 180.0 { d - 360.0 } else { d } }

// spin CCW until heading increases by target degrees (closed loop on compass)
fn spin_by(target_deg: f64) {
    let h0 = heading();
    if h0.is_nan() { return; }
    let deadline = Instant::now() + Duration::from_millis((target_deg.abs() / 30.0 * 1000.0 + 1500.0) as u64);
    let sign = if target_deg >= 0.0 { 1.0 } else { -1.0 };
    loop {
        drive(20.0 * sign, -20.0 * sign);
        let h = heading();
        if !h.is_nan() && angdiff(h, h0) * sign >= target_deg.abs() - 4.0 { break; }
        if Instant::now() > deadline { break; }
        thread::sleep(Duration::from_millis(60));
    }
    drive(0.0, 0.0);
    thread::sleep(Duration::from_millis(250));
}
fn run(l: f64, r: f64, ms: u64) {
    let end = Instant::now() + Duration::from_millis(ms);
    while Instant::now() < end { drive(l, r); thread::sleep(Duration::from_millis(40)); }
    drive(0.0, 0.0);
}
fn sample_d11() -> f64 {
    thread::sleep(Duration::from_millis(300));
    let mut acc = 0.0; let mut n = 0;
    let end = Instant::now() + Duration::from_millis(700);
    while Instant::now() < end {
        if let Some(v) = read_str("d11").and_then(|s| s.trim().parse::<f64>().ok()) { acc += v; n += 1; }
        thread::sleep(Duration::from_millis(80));
    }
    if n > 0 { acc / n as f64 } else { f64::NAN }
}
fn at_goal() -> bool { read_str("d3").map(|s| s.contains("goal=1")).unwrap_or(false) }

fn main() {
    evlog("START", "hillclimb begin");
    let mut leg_ms: u64 = 2000;
    let mut plateau = 0;
    let mut best_ever = f64::NAN;
    loop {
        if at_goal() {
            evlog("GOAL", "goal flag set — holding");
            radio("ATGOAL");
            let mut last_ping = Instant::now();
            loop {
                drive(0.0, 0.0);
                if last_ping.elapsed() > Duration::from_secs(2) { radio("ATGOAL"); last_ping = Instant::now(); }
                if let Some(m) = read_str("d10") { evlog("RX", &m); }
                thread::sleep(Duration::from_millis(100));
            }
        }
        if let Some(m) = read_str("d10") { evlog("RX", &m); }
        if let Some(st) = read_str("d3") { if st.contains("here=1") { evlog("HERE!", &st); radio("ISEEYOU"); } }
        radio("PING");

        // probe 4 headings: 0, +90, 180, -90 (relative)
        let mut results: Vec<(i32, f64)> = Vec::new();
        let base = sample_d11();
        evlog("BASE", &format!("d11={:.4}", base));
        for &(rel, turns) in [(0i32, 0i32), (90, 1), (180, 1), (270, 1)].iter() {
            if rel != 0 { spin_by(rel as f64); }
            run(20.0, 20.0, leg_ms);
            let v = sample_d11();
            run(-20.0, -20.0, leg_ms); // return
            if !v.is_nan() && !base.is_nan() { results.push((rel, v - base)); }
            evlog("PROBE", &format!("rel={} dd11={:.4}", rel, v - base));
        }
        // restore original heading: total spun = 0+90+90+90=270 CCW; spin -270 to undo... simpler: spin_by(-270) not needed for gradient; but keep continuity: undo
        spin_by(-270.0);
        if results.is_empty() { continue; }
        results.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap()); // MINIMIZE d11
        let (best_rel, best_dd) = results[0];
        evlog("BEST", &format!("rel={} dd11={:.4}", best_rel, best_dd));

        // plateau detection
        if !best_ever.is_nan() && best_dd.abs() < 0.006 { plateau += 1; } else { plateau = 0; }
        if plateau >= 2 {
            // refine with 45deg probes
            evlog("PLATEAU", "switching to fine probes");
            // probe +-45 deg
            spin_by(45.0); run(20.0, 20.0, leg_ms); let vp = sample_d11(); run(-20.0, -20.0, leg_ms); spin_by(-45.0);
            spin_by(-45.0); run(20.0, 20.0, leg_ms); let vm = sample_d11(); run(-20.0, -20.0, leg_ms); spin_by(45.0);
            let dp = vp - base; let dm = vm - base;
            evlog("FINE", &format!("+45:{:.4} -45:{:.4}", dp, dm));
            if dp.max(dm) < 0.006 { leg_ms = 3500; plateau = 0; } // try longer legs
        }

        // commit: move 2 legs in best direction
        if best_rel != 0 { spin_by(best_rel as f64); }
        run(20.0, 20.0, leg_ms);
        let v = sample_d11();
        evlog("MOVE", &format!("rel={} d11 after={:.4}", best_rel, v));
        if !v.is_nan() { if best_ever.is_nan() || v > best_ever { best_ever = v; } }
        if let Some(st) = read_str("d3") { evlog("ST", &st); }
    }
}
