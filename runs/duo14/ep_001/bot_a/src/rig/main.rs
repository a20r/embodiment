use std::fs::File;
use std::io::{Read, Write};
use std::time::{Duration, Instant};

fn read_port(p: &str) -> String {
    let mut s = String::new();
    // open read-only; these are read ports so should not block long
    if let Ok(mut f) = File::open(format!("/dev/robot/{}", p)) {
        let _ = f.read_to_string(&mut s);
    }
    s.trim().to_string()
}

fn write_port(p: &str, v: &str) -> bool {
    let t0 = Instant::now();
    let r = File::create(format!("/dev/robot/{}", p)).and_then(|mut f| f.write_all(format!("{}\n", v).as_bytes()));
    let ok = r.is_ok();
    if t0.elapsed() > Duration::from_millis(200) { println!("  (write {} slow {:?})", p, t0.elapsed()); }
    ok
}

fn snap() -> (String,String,String,String,String,String,String) {
    (read_port("d0"), read_port("d4"), read_port("d5"), read_port("d6"), read_port("d9"), read_port("d11"), read_port("d3"))
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    // trial: baseline_ms drive_l drive_r drive_ms post_ms
    let l = args[1].clone(); let r = args[2].clone();
    let dm: u64 = args[3].parse().unwrap_or(3000);
    println!("TRIAL d1={} d7={} dur={}ms", l, r, dm);
    // stop
    let _ = write_port("d1", "0"); let _ = write_port("d7", "0");
    std::thread::sleep(Duration::from_millis(1200));
    let (a0,a4,a5,a6,a9,a11,a3) = snap();
    println!("BASE d0={} d4={} d5={} d6={} d9={} d11={} {}", a0,a4,a5,a6,a9,a11,a3);
    let t0 = Instant::now();
    while t0.elapsed().as_millis() < dm as u128 {
        write_port("d1", &l); write_port("d7", &r);
        std::thread::sleep(Duration::from_millis(50));
    }
    let (b0,b4,b5,b6,b9,b11,b3) = snap();
    println!("END  d0={} d4={} d5={} d6={} d9={} d11={} {}", b0,b4,b5,b6,b9,b11,b3);
    std::thread::sleep(Duration::from_millis(1500));
    let (c0,c4,c5,c6,c9,c11,c3) = snap();
    println!("POST d0={} d4={} d5={} d6={} d9={} d11={} {}", c0,c4,c5,c6,c9,c11,c3);
}
