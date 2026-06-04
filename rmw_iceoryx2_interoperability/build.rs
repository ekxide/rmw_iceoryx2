use std::path::Path;

// The canonical type name lives in `src/constants.rs`; include it so the value injected into the
// generated header has a single source of truth (cbindgen itself cannot emit string constants).
#[path = "src/constants.rs"]
mod constants;

// Generates the C header only when an output path is requested (by the cmake/colcon build);
// plain `cargo build` for Rust consumers skips it.
fn main() {
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/constants.rs");
    println!("cargo:rerun-if-changed=cbindgen.toml");
    println!("cargo:rerun-if-env-changed=RMW_IOX2_INTEROP_HEADER_OUT");

    let output = match std::env::var("RMW_IOX2_INTEROP_HEADER_OUT") {
        Ok(path) => path,
        Err(_) => return,
    };

    let crate_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();

    let mut config = cbindgen::Config::from_root_or_default(&crate_dir);
    config.after_includes = Some(format!(
        "\nnamespace rmw_iceoryx2_interoperability {{\nconstexpr const char *MESSAGE_INFO_HEADER_TYPE_NAME = \"{}\";\n}}\n",
        constants::MESSAGE_INFO_HEADER_TYPE_NAME
    ));

    if let Some(parent) = Path::new(&output).parent() {
        std::fs::create_dir_all(parent).unwrap();
    }

    cbindgen::Builder::new()
        .with_crate(&crate_dir)
        .with_config(config)
        .generate()
        .expect("failed to generate C header")
        .write_to_file(&output);
}
