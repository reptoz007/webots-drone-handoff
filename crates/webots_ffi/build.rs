use std::env;
use std::path::PathBuf;

fn main() {
    let webots_home = env::var("WEBOTS_HOME")
        .unwrap_or_else(|_| r"C:\Users\harsh\AppData\Local\Programs\Webots".to_string());
    let lib_dir = PathBuf::from(&webots_home).join("lib").join("controller");
    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=dylib=Controller");
    println!("cargo:rerun-if-env-changed=WEBOTS_HOME");
}
