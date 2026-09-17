use std::fs::File;
use std::io::{Read, Write};
use std::time::{Duration, Instant};

fn rp(p: &str) -> String {
    let mut s = String::new();
    if let Ok(mut f) = File::open(format!("/dev/robot/{}", p)) {
        let _ = f.read_to_string(&mut s);
    }
    s.trim().to_string()
}
fn wp(p: &str, v: f64) {
    if let Ok(mut f) = File::create(format!("/dev/robot/{}", p)) {
        let _ = f.write_all(format!("{}\n", v).as_bytes());
    }
}
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let v: f64 = a[1].parse().unwrap();
    let secs: f64 = a[2].parse().unwrap();
    let dir: i32 = a[3].parse().unwrap();
    let (l, r) = (v * dir as f64, -v * dir as f64);
    let mut log = File::create("/tmp/spin2.log").unwrap();
    wp("d1", 0.0); wp("d7", 0.0);
    std::thread::sleep(Duration::from_millis(400));
    let h0 = rp("d4");
    let t0 = Instant::now();
    while t0.elapsed() < Duration::from_secs_f64(secs) {
        wp("d1", l); wp("d7", r);
        let line = format!("{} {} {} {} {}\n", t0.elapsed().as_millis(), rp("d4"), rp("d6"), rp("d9"), rp("d2"));
        let _ = log.write_all(line.as_bytes());
        std::thread::sleep(Duration::from_millis(80));
    }
    wp("d1", 0.0); wp("d7", 0.0);
    println!("h0={} h1={}", h0, rp("d4"));
}
