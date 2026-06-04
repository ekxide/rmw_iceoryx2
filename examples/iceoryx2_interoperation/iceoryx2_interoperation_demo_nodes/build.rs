// Link/rpath the rosidl-generated C libraries the message crate refers to.
fn main() {
    let prefix_path = std::env::var("AMENT_PREFIX_PATH")
        .expect("AMENT_PREFIX_PATH not set - source the ROS 2 workspace before building");
    for prefix in prefix_path.split(':') {
        let lib = format!("{prefix}/lib");
        println!("cargo:rustc-link-search=native={lib}");
        println!("cargo:rustc-link-arg=-Wl,-rpath,{lib}");
    }
}
