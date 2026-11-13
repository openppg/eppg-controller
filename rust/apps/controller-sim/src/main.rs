use controller_sim::{parse_scenario, Simulator};
use std::{env, fs, io::BufWriter};
fn main() {
    if let Err(error) = run() {
        eprintln!("controller-sim: {error}");
        std::process::exit(1);
    }
}
fn run() -> Result<(), String> {
    let mut args = env::args().skip(1);
    let mut scenario = None;
    let mut trace = None;
    let mut duration = 10000u32;
    let mut quiet = false;
    let mut framebuffer = None;
    let mut dark = false;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--scenario" => scenario = Some(args.next().ok_or("missing scenario file")?),
            "--duration" => {
                duration = args
                    .next()
                    .ok_or("missing duration")?
                    .parse()
                    .map_err(|_| "invalid duration")?
            }
            "--trace" => trace = Some(args.next().ok_or("missing trace path")?),
            "--quiet" => quiet = true,
            "--framebuffer" => {
                framebuffer = Some(args.next().ok_or("missing framebuffer BMP path")?)
            }
            "--dark" => dark = true,
            "--help" => {
                println!("controller-sim --scenario FILE [--duration MS] [--trace JSONL] [--framebuffer BMP] [--dark] [--quiet]");
                return Ok(());
            }
            _ => return Err(format!("unknown argument {arg}")),
        }
    }
    if duration > 3_600_000 {
        return Err("duration exceeds one-hour simulation limit".into());
    }
    let mut sim = Simulator::new()?;
    let text = fs::read_to_string(scenario.ok_or("--scenario FILE is required")?)
        .map_err(|e| e.to_string())?;
    for event in parse_scenario(&text)? {
        sim.schedule(event)?;
    }
    let mut output = trace
        .map(|path| fs::File::create(path).map(BufWriter::new))
        .transpose()
        .map_err(|e| e.to_string())?;
    let writer = output.as_mut().map(|w| w as &mut dyn std::io::Write);
    sim.run(duration, writer)?;
    if let Some(path) = framebuffer {
        sim.render_bmp(std::path::Path::new(&path), dark)?;
    }
    if !quiet {
        for line in &sim.logs {
            eprintln!("{line}");
        }
    }
    println!("{}", sim.snapshot());
    Ok(())
}
