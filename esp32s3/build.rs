extern crate prost_build;

fn main() {
    prost_build::compile_protos(&["src/data.proto"],
                                &["src/"]).unwrap();
    println!("cargo:rustc-link-arg-bins=-Tlinkall.x");
}