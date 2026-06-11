//! Latency collection and reporting, rendering the same summary as the
//! ROS 2 benchmark nodes (`stats.hpp`).

/// Formats a duration given in nanoseconds, picking a readable unit.
pub fn format_latency(nanoseconds: i64) -> String {
    if nanoseconds < 1_000 {
        format!("{nanoseconds} ns")
    } else if nanoseconds < 1_000_000 {
        format!("{:.1} µs", nanoseconds as f64 / 1_000.0)
    } else {
        format!("{:.2} ms", nanoseconds as f64 / 1_000_000.0)
    }
}

/// A bar proportional to value/max, using eighth-blocks for the remainder.
fn bar(value: i64, max: i64) -> String {
    const FULL_WIDTH: f64 = 38.0;
    const EIGHTHS: [&str; 8] = ["", "▏", "▎", "▍", "▌", "▋", "▊", "▉"];
    if max <= 0 {
        return String::new();
    }
    let cells = value as f64 / max as f64 * FULL_WIDTH;
    let mut out = "█".repeat(cells as usize);
    out.push_str(EIGHTHS[((cells - cells.floor()) * 8.0) as usize]);
    if out.is_empty() && value > 0 {
        out.push('▏'); // a nonzero value always gets a visible bar
    }
    out
}

/// Collects one-way latencies and renders a summary with the percentile
/// distribution drawn as proportional bars, e.g.:
///
/// ```text
/// REPORT · iceoryx2 subscriber
/// samples 10000/10000 · lost 0 · warmup 100
///
///   min    4.1 µs ▏
///   p50    8.9 µs ███▎
///   p99   21.0 µs ████████▏
///   max   98.2 µs ██████████████████████████████████████
///
///   mean   9.4 µs
/// ```
pub struct LatencyRecorder {
    warmup: u64,
    received: u64,
    latencies: Vec<i64>,
}

impl LatencyRecorder {
    pub fn new(warmup: u64, expected: u64) -> Self {
        Self {
            warmup,
            received: 0,
            latencies: Vec::with_capacity(expected as usize),
        }
    }

    /// Records one received sample; the first `warmup` samples are counted
    /// but excluded from the latency statistics.
    pub fn record(&mut self, latency_nanoseconds: i64) {
        self.received += 1;
        if self.received <= self.warmup {
            return;
        }
        self.latencies.push(latency_nanoseconds);
    }

    pub fn received(&self) -> u64 {
        self.received
    }

    pub fn report(&self, label: &str, expected: u64) -> String {
        let colour = "\x1b[1;35m"; // bold magenta
        let reset = "\x1b[0m";

        let mut out = format!("\n{colour}REPORT{reset} · {label}\n");
        out.push_str(&format!(
            "samples {}/{} · lost {} · warmup {}\n",
            self.received,
            expected,
            expected - self.received,
            self.warmup
        ));

        if self.latencies.is_empty() {
            return out;
        }

        let mut sorted = self.latencies.clone();
        sorted.sort_unstable();
        let percentile = |fraction: f64| -> i64 {
            let index = (fraction * (sorted.len() - 1) as f64) as usize;
            sorted[index]
        };
        let max = sorted[sorted.len() - 1];
        let mut row = |out: &mut String, name: &str, value: i64, with_bar: bool| {
            out.push_str(&format!("  {name:<5}{:>8}", format_latency(value)));
            if with_bar {
                out.push(' ');
                out.push_str(&bar(value, max));
            }
            out.push('\n');
        };

        out.push('\n');
        row(&mut out, "min", sorted[0], true);
        row(&mut out, "p50", percentile(0.50), true);
        row(&mut out, "p90", percentile(0.90), true);
        row(&mut out, "p99", percentile(0.99), true);
        row(&mut out, "max", max, true);
        out.push('\n');
        let sum: i64 = sorted.iter().sum();
        row(&mut out, "mean", sum / sorted.len() as i64, false);
        out
    }
}
