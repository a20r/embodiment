use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::thread;
use std::time::{Duration, Instant};

fn set(p: &str, v: &str) { if let Ok(mut f) = File::create(format!("/dev/robot/{}", p)) { let _ = writeln!(f, "{}", v); } }
fn drive(l: f64, r: f64) { set("d1", &format!("{}", l)); set("d7", &format!("{}", r)); }
fn stop() { for _ in 0..10 { drive(0.0, 0.0); thread::sleep(Duration::from_millis(30)); } }
fn radio(m: &str) { if let Ok(mut f) = File::create("/dev/robot/d8") { let _ = writeln!(f, "{}", m); } }
fn evlog(tag: &str, msg: &str) {
    if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open("/tmp/climb.log") {
        let _ = writeln!(f, "{} {}", tag, msg);
    }
}
fn read_str(p: &str) -> Option<String> {
    if let Ok(f) = File::open(format!("/dev/robot/{}", p)) {
        let mut r = BufReader::new(f);
        let mut s = String::new();
        if r.read_line(&mut s).is_ok() && !s.trim().is_empty() { return Some(s.trim().to_string()); }
    }
    None
}
fn heading() -> Option<f64> { read_str("d4").and_then(|s| s.trim().parse::<f64>().ok()) }
fn norm(a: f64) -> f64 { let mut x = a % 360.0; if x < 0.0 { x += 360.0; } x }
fn angdiff(a: f64, b: f64) -> f64 { let d = norm(a - b); if d > 180.0 { d - 360.0 } else { d } }

fn spin_to(target: f64) {
    let deadline = Instant::now() + Duration::from_millis(4000);
    loop {
        let h = match heading() { Some(h) => h, None => { thread::sleep(Duration::from_millis(50)); continue; } };
        let err = angdiff(target, h); // + need CCW
        if err.abs() <= 5.0 { break; }
        if Instant::now() > deadline { break; }
        if err > 0.0 { drive(20.0, -20.0); } else { drive(-20.0, 20.0); }
        thread::sleep(Duration::from_millis(60));
    }
    stop();
    thread::sleep(Duration::from_millis(200));
}
fn read_lidar() -> Vec<f64> {
    let mut out = vec![1.45f64; 16];
    if let Ok(f) = File::open("/dev/robot/d2") {
        let mut r = BufReader::new(f);
        let mut s = String::new();
        if r.read_line(&mut s).is_ok() {
            for (i, tok) in s.trim().split(',').enumerate() {
                if i < 16 {
                    let v: f64 = tok.trim().parse().unwrap_or(1.45);
                    out[i] = if v < 0.0 { 1.45 } else { v };
                }
            }
        }
    }
    out
}
fn run(l: f64, r: f64, ms: u64) {
    let end = Instant::now() + Duration::from_millis(ms);
    while Instant::now() < end {
        let lid = read_lidar();
        let front_min = (3..=8).map(|i| lid[i]).fold(f64::INFINITY, f64::min);
        if front_min < 0.28 { evlog("BLOCKED", &format!("front={:.2}", front_min)); break; }
        drive(l, r); thread::sleep(Duration::from_millis(40));
    }
    stop();
}
fn sample_d11() -> f64 {
    thread::sleep(Duration::from_millis(250));
    let mut acc = 0.0; let mut n = 0;
    let end = Instant::now() + Duration::from_millis(800);
    while Instant::now() < end {
        if let Some(v) = read_str("d11").and_then(|s| s.trim().parse::<f64>().ok()) { acc += v; n += 1; }
        thread::sleep(Duration::from_millis(70));
    }
    if n > 0 { acc / n as f64 } else { f64::NAN }
}
fn at_goal() -> bool { read_str("d3").map(|s| s.contains("goal=1")).unwrap_or(false) }

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let legs: u64 = args.get(1).and_then(|a| a.parse().ok()).unwrap_or(10);
    let leg_ms: u64 = args.get(2).and_then(|a| a.parse().ok()).unwrap_or(4000);
    evlog("SWEEP", "begin lawn-mower");
    let mut best = 0.0f64;
    for n in 0..legs {
        run(20.0, 20.0, leg_ms);
        let v = sample_d11();
        evlog("SW", &format!("n={} d11={:.4} best={:.4}", n, v, best));
        if !v.is_nan() && v > best { best = v; }
        // 90 deg turn
        let h = heading().unwrap_or(0.0);
        let tgt = if n % 2 == 0 { norm(h + 90.0) } else { norm(h - 90.0) };
        spin_to(tgt);
        run(20.0, 20.0, 900); // offset
        let h2 = heading().unwrap_or(0.0);
        let tgt2 = if n % 2 == 0 { norm(h2 + 90.0) } else { norm(h2 - 90.0) };
        spin_to(tgt2);
        if at_goal() { evlog("GOAL", "found during sweep"); loop { stop(); radio("A1 ATGOAL"); thread::sleep(Duration::from_millis(800)); } }
    }
    evlog("SWEEP", &format!("done best={:.4}", best));
    // finish: hold
    loop { stop(); thread::sleep(Duration::from_millis(500)); }
}
