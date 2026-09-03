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
fn set(port: &str, v: &str) {
    if let Ok(mut f) = File::create(format!("/dev/robot/{}", port)) { let _ = writeln!(f, "{}", v); }
}
fn drive(l: f64, r: f64) {
    let ls = format!("{}", l); let rs = format!("{}", r);
    set("d1", &ls); set("d7", &rs);
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let duration: u64 = if args.len() > 1 { args[1].parse().unwrap() } else { 600 };
    let v: f64 = if args.len() > 2 { args[2].parse().unwrap() } else { 20.0 };

    let ports = ["d0","d2","d3","d4","d5","d6","d9","d10","d11"];
    let (tx, rx) = mpsc::channel();
    let t0 = Instant::now();
    for p in ports { let tx = tx.clone(); thread::spawn(move || reader(p, t0, tx)); }
    let logf = OpenOptions::new().create(true).write(true).truncate(true).open("/tmp/explore.log").unwrap();
    let _logger = thread::spawn(move || {
        let mut out = logf;
        while let Ok((t,p,l)) = rx.recv() { let _ = writeln!(out, "{:.3} {} {}", t, p, l); }
    });

    let deadline = t0.elapsed() + Duration::from_secs(duration);
    let mut last_lidar: Vec<f64> = vec![0.0;16];
    let mut static_secs = 0.0;
    let mut maneuver = 0; // 0=forward,1=reverse,2=spinL,3=spinR
    let mut maneuver_until = t0.elapsed();
    let mut last_ping = t0.elapsed();
    let mut rx_poll = t0.elapsed();
    let mut arrived = false;

    while t0.elapsed() < deadline {
        // read latest lidar + status from a shared snapshot via channel is complex; do inline reads instead
        let lid = read_lidar();
        let status = read_line("d3");

        if let Some(st) = status {
            if st.contains("goal=1") && !arrived {
                arrived = true;
                drive(0.0,0.0);
                radio("ATGOAL");
                log(t0, "STATE", "GOAL REACHED - holding");
                // hold position, keep pinging
            }
        }
        if arrived {
            drive(0.0,0.0);
            if t0.elapsed() - last_ping > Duration::from_secs(2) { radio("ATGOAL"); last_ping = t0.elapsed(); }
            thread::sleep(Duration::from_millis(100));
            continue;
        }

        // radio: send ping every 3s
        if t0.elapsed() - last_ping > Duration::from_secs(3) {
            radio("PING");
            last_ping = t0.elapsed();
        }
        // radio: poll receives
        if t0.elapsed() - rx_poll > Duration::from_millis(200) {
            if let Some(msg) = try_read("d10") { log(t0, "RX", &msg); }
            rx_poll = t0.elapsed();
        }

        // lidar-based reflexes
        let front_min = (4..=9).map(|i| lid[i]).fold(f64::INFINITY, f64::min);
        let motion: f64 = (0..16).map(|i| (lid[i]-last_lidar[i]).abs()).sum::<f64>() / 16.0;
        last_lidar = lid.clone();
        if motion < 0.004 { static_secs += 0.1; } else { static_secs = 0.0; }

        if t0.elapsed() >= maneuver_until {
            maneuver = 0;
        }
        if static_secs > 4.0 {
            // stuck: pick an escape
            maneuver = 1 + (static_secs as u64 % 3) as i32;
            maneuver_until = t0.elapsed() + Duration::from_secs(1);
            static_secs = 0.0;
        } else if front_min < 0.32 && maneuver == 0 {
            maneuver = if front_min < 0.22 { 2 } else { 3 };
            maneuver_until = t0.elapsed() + Duration::from_millis(900);
        }

        match maneuver {
            0 => drive(v, v),
            1 => drive(-v, -v),
            2 => drive(v, -v),
            _ => drive(-v, v),
        }
        thread::sleep(Duration::from_millis(100));
    }
    drive(0.0,0.0);
    log(t0, "STATE", "exploration duration over");
    unsafe { std::process::exit(0); }
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
    out
}
fn read_line(port: &str) -> Option<String> {
    if let Ok(f) = File::open(format!("/dev/robot/{}", port)) {
        let mut r = BufReader::new(f);
        let mut s = String::new();
        if r.read_line(&mut s).is_ok() && !s.trim().is_empty() { return Some(s.trim().to_string()); }
    }
    None
}
fn try_read(port: &str) -> Option<String> { read_line(port) }
fn radio(msg: &str) {
    if let Ok(mut f) = File::create("/dev/robot/d8") { let _ = writeln!(f, "{}", msg); }
}
fn log(t0: Instant, tag: &str, msg: &str) {
    if let Ok(mut f) = OpenOptions::new().create(true).append(true).open("/tmp/explore_events.log") {
        let _ = writeln!(f, "{:.2} {} {}", t0.elapsed().as_secs_f64(), tag, msg);
    }
}
