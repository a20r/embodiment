use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, Write};
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

fn reader(port: &'static str, t0: Instant, tx: mpsc::Sender<(f64, String, String)>) {
    loop {
        if let Ok(f) = File::open(format!("/dev/robot/{}", port)) {
            let mut r = BufReader::new(f);
            let mut line = String::new();
            match r.read_line(&mut line) {
                Ok(n) if n > 0 => { let _ = tx.send((t0.elapsed().as_secs_f64(), port.to_string(), line.trim().to_string())); },
                _ => {},
            }
        } else { thread::sleep(Duration::from_millis(20)); }
    }
}
fn set(port: &str, v: &str) { if let Ok(mut f) = File::create(format!("/dev/robot/{}", port)) { let _ = writeln!(f, "{}", v); } }
fn drive(l: f64, r: f64) { set("d1", &format!("{}", l)); set("d7", &format!("{}", r)); }
fn radio(msg: &str) { if let Ok(mut f) = File::create("/dev/robot/d8") { let _ = writeln!(f, "{}", msg); } }
fn evlog(t0: Instant, tag: &str, msg: &str) {
    if let Ok(mut f) = OpenOptions::new().create(true).append(true).open("/tmp/nav_events.log") {
        let _ = writeln!(f, "{:.2} {} {}", t0.elapsed().as_secs_f64(), tag, msg);
    }
}
fn read_lidar() -> Vec<f64> {
    let mut out = vec![0.0; 16];
    if let Ok(f) = File::open("/dev/robot/d2") {
        let mut r = BufReader::new(f);
        let mut s = String::new();
        if r.read_line(&mut s).is_ok() {
            for (i, tok) in s.trim().split(',').enumerate() {
                if i < 16 { out[i] = tok.trim().parse().unwrap_or(-1.0); }
            }
        }
    }
    for v in out.iter_mut() { if *v < 0.0 { *v = 1.45; } } // -1 = no return = far
    out
}
fn read_str(port: &str) -> Option<String> {
    if let Ok(f) = File::open(format!("/dev/robot/{}", port)) {
        let mut r = BufReader::new(f);
        let mut s = String::new();
        if r.read_line(&mut s).is_ok() && !s.trim().is_empty() { return Some(s.trim().to_string()); }
    }
    None
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let duration: u64 = if args.len() > 1 { args[1].parse().unwrap() } else { 900 };
    let v: f64 = 20.0;
    let (tx, rx) = mpsc::channel();
    let t0 = Instant::now();
    let ports = ["d0","d2","d3","d4","d5","d6","d9","d10","d11"];
    for p in ports { let tx = tx.clone(); thread::spawn(move || reader(p, t0, tx)); }
    let _logger = thread::spawn(move || {
        let mut out = OpenOptions::new().create(true).write(true).truncate(true).open("/tmp/nav.log").unwrap();
        while let Ok((t,p,l)) = rx.recv() { let _ = writeln!(out, "{:.3} {} {}", t, p, l); }
    });

    // initial escape: 3 cycles of reverse + CCW spin
    for _ in 0..3 {
        drive(-v, -v); thread::sleep(Duration::from_millis(1200));
        drive(v, -v);  thread::sleep(Duration::from_millis(1100));
    }
    evlog(t0, "ESC", "escape cycles done");
    let deadline = t0.elapsed() + Duration::from_secs(duration);
    let mut last = vec![0.0f64;16];
    let mut static_t = 0.0f64;
    let mut last_ping = Instant::now();
    let mut last_poll = Instant::now();
    let mut arrived = false;
    let mut seq: u64 = 0;

    while t0.elapsed() < deadline {
        seq += 1;
        let lid = read_lidar();
        // front = rays 4..=8
        let front_min = (4..=8).map(|i| lid[i]).fold(f64::INFINITY, f64::min);
        let front_avg = (4..=8).map(|i| lid[i]).sum::<f64>() / 5.0;
        // rotate-to-gap: find best ray in forward arc (rays 2..=8), steer to center it
        // ray k bearing = (k-5)*22.5 deg. forward = ray 5.
        let mut best_k = 5; let mut best_d = -1.0;
        for k in 2..=8 {
            let d = lid[k];
            // score: distance minus penalty for being off-center
            let score = d - 0.02 * ((k as f64) - 5.0).abs();
            if d > best_d { best_d = d; best_k = k; }
        }
        let err = (best_k as f64) - 5.0; // rays off center; + = need CCW
        let mut u = 9.0 * err * (front_avg.max(0.2));
        if u > 20.0 { u = 20.0; } if u < -20.0 { u = -20.0; }

        // stuck detection
        let motion = (0..16).map(|i| (lid[i]-last[i]).abs()).sum::<f64>() / 16.0;
        last = lid.clone();
        static_t = if motion < 0.003 { static_t + 0.1 } else { 0.0 };

        // collision check
        if let Some(d0) = read_str("d0") { if d0.trim() == "1" {
            evlog(t0, "BUMP", "collision!");
            drive(-v, -v);
            thread::sleep(Duration::from_millis(600));
            drive(v, -v);
            thread::sleep(Duration::from_millis(700));
        }}

        // status
        if let Some(st) = read_str("d3") {
            if st.contains("goal=1") && !arrived {
                arrived = true;
                evlog(t0, "GOAL", "goal flag set!");
                radio("ATGOAL");
            }
            if st.contains("here=1") { evlog(t0, "HERE", &st); }
        }
        if arrived {
            drive(0.0, 0.0);
            if last_ping.elapsed() > Duration::from_secs(2) { radio("ATGOAL"); last_ping = Instant::now(); }
            if last_poll.elapsed() > Duration::from_millis(150) {
                if let Some(m) = read_str("d10") { evlog(t0, "RX", &m); }
                last_poll = Instant::now();
            }
            thread::sleep(Duration::from_millis(80));
            continue;
        }

        // radio
        if last_ping.elapsed() > Duration::from_secs(3) { radio("PING"); last_ping = Instant::now(); }
        if last_poll.elapsed() > Duration::from_millis(150) {
            if let Some(m) = read_str("d10") { evlog(t0, "RX", &m); }
            last_poll = Instant::now();
        }

        if static_t > 4.0 {
            evlog(t0, "STUCK", "escape maneuver");
            drive(-v, -v); thread::sleep(Duration::from_millis(800));
            drive(v, -v);  thread::sleep(Duration::from_millis(900));
            static_t = 0.0;
        } else if front_min < 0.15 {
            evlog(t0, "PINNED", "front_min<0.15");
            drive(-v, -v); thread::sleep(Duration::from_millis(900));
            drive(v, -v);  thread::sleep(Duration::from_millis(800));
        } else if front_min < 0.30 {
            // hard avoid: turn toward open side
            let left_open = (9..=12).map(|i| lid[i]).sum::<f64>();
            let right_open = (0..=3).map(|i| lid[i]).sum::<f64>();
            if left_open > right_open { drive(v, -v); } else { drive(-v, v); }
            thread::sleep(Duration::from_millis(250));
        } else if front_avg < 0.45 {
            u = u.signum() * 12.0; // slow + strong steer
            drive(v - u, v + u);
            thread::sleep(Duration::from_millis(100));
        } else {
            drive(v - u, v + u);
            thread::sleep(Duration::from_millis(100));
        }

        if seq % 10 == 0 {
            let d9 = read_str("d9").unwrap_or_default();
            let d6 = read_str("d6").unwrap_or_default();
            let d11 = read_str("d11").unwrap_or_default();
            let st = read_str("d3").unwrap_or_default();
            evlog(t0, "O", &format!("d9={} d6={} d11={} d4={} {} | f={:.2} e={:.1}", d9, d6, d11, read_str("d4").unwrap_or_default(), st, front_avg, err));
        }
    }
    drive(0.0, 0.0);
    evlog(t0, "END", "duration over");
    unsafe { std::process::exit(0); }
}
