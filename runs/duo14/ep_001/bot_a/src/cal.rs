use std::fs::{File, OpenOptions};
use std::io::{Read, Write, BufWriter};
use std::time::{Duration, Instant};

fn rp(p: &str) -> String {
    let mut s = String::new();
    if let Ok(mut f) = File::open(format!("/dev/robot/{}", p)) { let _ = f.read_to_string(&mut s); }
    s.trim().to_string()
}
fn wp(p: &str, v: f64) {
    if let Ok(mut f) = File::create(format!("/dev/robot/{}", p)) {
        let _ = f.write_all(format!("{}\n", v).as_bytes());
    }
}
fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mode = args.get(1).map(|s| s.as_str()).unwrap_or("spin");
    let secs: f64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(8.0);
    let mut log = BufWriter::new(OpenOptions::new().create(true).append(true).open("/tmp/cal.log").unwrap());
    wp("d1", 0.0); wp("d7", 0.0);
    std::thread::sleep(Duration::from_millis(500));
    let (l, r) = match mode {
        "spinl" => (1.5, -1.5), "spinr" => (-1.5, 1.5),
        "fwd" => (1.0, 1.0),
        _ => (0.0, 0.0),
    };
    let t0 = Instant::now();
    let dur = Duration::from_secs_f64(secs);
    let mut n = 0;
    while t0.elapsed() < dur {
        wp("d1", l); wp("d7", r);
        let line = format!("{} {} {} {} {} {} {} {} {} {}\n",
            t0.elapsed().as_millis(), mode, l, r,
            rp("d4"), rp("d6"), rp("d9"), rp("d0"), rp("d5"), rp("d2"));
        let _ = log.write_all(line.as_bytes()); let _ = log.flush();
        n += 1;
        std::thread::sleep(Duration::from_millis(60));
    }
    wp("d1", 0.0); wp("d7", 0.0);
    // post
    for _ in 0..10 { std::thread::sleep(Duration::from_millis(100)); }
    let line = format!("{} END {} {} {} {} {} {} {} {} {}\n",
        t0.elapsed().as_millis(), 0.0, 0.0, rp("d4"), rp("d6"), rp("d9"), rp("d0"), rp("d5"), rp("d2"), "");
    let _ = log.write_all(line.as_bytes());
    println!("done {} samples", n);
}
