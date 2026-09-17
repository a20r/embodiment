use std::fs::File;
use std::io::Read;
use std::time::Duration;

fn rp(p: &str) -> String {
    let mut s = String::new();
    if let Ok(mut f) = File::open(format!("/dev/robot/{}", p)) {
        let _ = f.read_to_string(&mut s);
    }
    s.trim().to_string()
}
fn pf(s: &str) -> f64 { s.parse().unwrap_or(-1.0) }

fn main() {
    let mut acc = [0.0f64; 16]; let mut n = [0u32; 16];
    let mut hd: Vec<f64> = Vec::new();
    for _ in 0..12 {
        let raw: Vec<f64> = rp("d2").split(',').map(pf).collect();
        if raw.len() == 16 { for k in 0..16 { if raw[k] >= 0.0 { acc[k] += raw[k]; n[k] += 1; } } }
        let h = pf(&rp("d4"));
        if h >= 0.0 { hd.push(h); }
        std::thread::sleep(Duration::from_millis(60));
    }
    let avg: Vec<f64> = (0..16).map(|k| if n[k] > 0 { acc[k] / n[k] as f64 } else { -1.0 }).collect();
    let hmean = if !hd.is_empty() { hd.iter().sum::<f64>() / hd.len() as f64 } else { -1.0 };
    println!("hdg={:.1}", hmean);
    println!("beams: {}", avg.iter().map(|v| format!("{:.2}", v)).collect::<Vec<_>>().join(","));
    let mut mk = 0; let mut mr = -1.0;
    for k in 0..16 { if avg[k] > mr { mr = avg[k]; mk = k; } }
    println!("maxbeam={}@{:.2}", mk, mr);
}
