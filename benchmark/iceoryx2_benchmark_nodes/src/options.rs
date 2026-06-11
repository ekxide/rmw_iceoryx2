//! CLI flags shared by the benchmark binaries, mirroring the flags of the ROS 2 nodes.

pub struct Options {
    /// Publish rate in Hz (publisher only).
    pub rate: f64,
    /// Total samples in the run.
    pub count: u64,
    /// Received samples excluded from the statistics.
    pub warmup: u64,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            rate: 1000.0,
            count: 10000,
            warmup: 100,
        }
    }
}

/// Parses the benchmark flags, ignoring anything else.
pub fn parse() -> Options {
    let mut options = Options::default();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--rate" => {
                if let Some(value) = args.next() {
                    options.rate = value.parse().expect("--rate expects a number");
                }
            }
            "--count" => {
                if let Some(value) = args.next() {
                    options.count = value.parse().expect("--count expects an integer");
                }
            }
            "--warmup" => {
                if let Some(value) = args.next() {
                    options.warmup = value.parse().expect("--warmup expects an integer");
                }
            }
            "--help" | "-h" => {
                println!("usage: [--rate <hz>] [--count <n>] [--warmup <n>]");
                std::process::exit(0);
            }
            _ => {}
        }
    }
    options
}
