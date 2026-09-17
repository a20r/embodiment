use std::fs::File;
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

fn main() {
    let ports = ["d0","d2","d3","d4","d5","d6","d9","d11"];
    let (tx, rx) = mpsc::channel();
    let t0 = Instant::now();
    for p in ports { let tx = tx.clone(); thread::spawn(move || reader(p, t0, tx)); }
    let _logger = thread::spawn(move || {
        let mut out = File::create("/tmp/probe.log").unwrap();
        while let Ok((t,p,l)) = rx.recv() { writeln!(out, "{:.3} {} {}", t, p, l).unwrap(); }
    });
    let mut drive = |l: &str, r: &str, ms: u64, t0: Instant| {
        let end = t0.elapsed() + Duration::from_millis(ms);
        while t0.elapsed() < end { set("d1", l); set("d7", r); thread::sleep(Duration::from_millis(40)); }
        set("d1","0"); set("d7","0");
        println!("PHASE l={} r={} ended at {:.2}", l, r, t0.elapsed().as_secs_f64());
    };
    drive("0","0",2000,t0);
    drive("20","-20",1200,t0);
    drive("0","0",1500,t0);
    drive("-20","20",1200,t0);
    drive("0","0",1500,t0);
    drive("20","20",1500,t0);
    drive("0","0",1500,t0);
    drive("-20","-20",1500,t0);
    drive("0","0",1000,t0);
    thread::sleep(Duration::from_millis(500));
    unsafe { std::process::exit(0); }
}
