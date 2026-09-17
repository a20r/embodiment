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
    evlog("START", "climb v2 absolute-heading");
    let mut best_d11 = sample_d11();
    let mut base_h = heading().unwrap_or(0.0);
    evlog("BASE0", &format!("d11={:.4} h={:.1}", best_d11, base_h));
    let mut leg: u64 = std::env::args().nth(1).and_then(|a| a.parse().ok()).unwrap_or(2500);
    loop {
        // escape if pinned
        let fm = { let lid = read_lidar(); (3..=8).map(|i| lid[i]).fold(f64::INFINITY, f64::min) };
        if fm < 0.35 {
            evlog("ESCAPE", &format!("front={:.2}", fm));
            for _ in 0..4 {
                run(-20.0, -20.0, 1100);
                run(20.0, -20.0, 1300);
                let lid = read_lidar();
                let f2 = (3..=8).map(|i| lid[i]).fold(f64::INFINITY, f64::min);
                if f2 > 0.5 { break; }
            }
        }
        if at_goal() {
            evlog("GOAL", "GOAL FLAG - holding forever, radio burst");
            loop {
                stop();
                radio("A1 ATGOAL - come to me, holding");
                if let Some(m) = read_str("d10") { evlog("RX", &m); }
                thread::sleep(Duration::from_millis(700));
            }
        }
        if let Some(m) = read_str("d10") { evlog("RX", &m); radio("A1 climbing d11; state ok"); }
        // probe 4 absolute headings based on current heading
        let h0 = heading().unwrap_or(0.0);
        let mut results: Vec<(f64, f64, f64)> = Vec::new(); // (abs_heading, dd11, abs_after)
        for k in 0..8 {
            let target = norm(h0 + 45.0 * k as f64);
            spin_to(target);
            run(20.0, 20.0, leg);
            let v = sample_d11();
            let h_after = heading().unwrap_or(f64::NAN);
            run(-20.0, -20.0, leg);
            if !v.is_nan() { results.push((target, v - best_d11, h_after)); }
            evlog("PROBE", &format!("abs={:.0} dd={:.4} hAfter={:.1}", target, v - best_d11, h_after));
        }
        if results.is_empty() { continue; }
        results.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        let (bt, bdd, _) = results[0];
        evlog("BEST", &format!("abs={:.0} dd={:.4}", bt, bdd));
        if bdd > 0.004 {
            spin_to(bt);
            run(20.0, 20.0, leg);
            let v = sample_d11();
            evlog("MOVE", &format!("abs={:.0} d11={:.4}", bt, v));
            if !v.is_nan() && v > best_d11 { best_d11 = v; }
            radio(&format!("A1 climbing d11={:.3}", best_d11));
        } else {
            // plateau: try longer legs in best direction
            spin_to(bt);
            run(20.0, 20.0, leg * 2);
            let v = sample_d11();
            evlog("PLATEAU_MOVE", &format!("d11={:.4}", v));
            if !v.is_nan() && v > best_d11 { best_d11 = v; }
            if leg < 4000 { leg += 500; }
            if best_d11 > 0.68 {
                evlog("FINESPIRAL", "engaging fine square search");
                let mut side = 600u64;
                for n in 0..12 {
                    run(20.0, 20.0, side);
                    let v = sample_d11();
                    evlog("FINE", &format!("n={} d11={:.4}", n, v));
                    if !v.is_nan() && v > best_d11 { best_d11 = v; side = 600; }
                    spin_to(norm(heading().unwrap_or(0.0) + 90.0));
                    if at_goal() { break; }
                }
            }
        }
    }
}
