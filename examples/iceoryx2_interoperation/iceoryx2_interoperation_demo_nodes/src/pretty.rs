//! Renders messages the same way the talker example's `pretty.hpp` does, so
//! both stacks of the demo produce visually identical output.

/// Direction of a message relative to the node printing it, which selects the
/// ANSI colour used to highlight the frame header.
pub enum Direction {
    Sent,
    Received,
}

/// Renders a message as a left-bordered, colour-headed frame. An optional
/// `meta` string is appended to the header line (the subscriber uses it for
/// the sequence number; the publisher leaves it empty). A leading newline
/// separates consecutive frames.
///
/// ```text
/// ╭─ RECV · seq 42
/// │  x             42
/// │  funky         34109.04
/// ╰────────────────────────────
/// ```
pub fn frame(direction: Direction, meta: &str, fields: &[(&str, String)]) -> String {
    let (label, colour) = match direction {
        Direction::Sent => ("SENT", "\x1b[1;32m"), // bold green
        Direction::Received => ("RECV", "\x1b[1;36m"), // bold cyan
    };
    let reset = "\x1b[0m";

    let mut out = format!("\n{colour}╭─ {label}{reset}");
    if !meta.is_empty() {
        out.push_str(" · ");
        out.push_str(meta);
    }
    out.push('\n');
    for (name, value) in fields {
        out.push_str(&format!("│  {name:<14}{value}\n"));
    }
    out.push_str("╰────────────────────────────");
    out
}

/// Formats a floating point value with two decimal places.
pub fn number(value: f64) -> String {
    format!("{value:.2}")
}
