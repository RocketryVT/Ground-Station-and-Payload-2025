extern crate prost_build;

fn main() {
    prost_build::compile_protos(&["../esp32s3/src/data.proto"],
                                &["../esp32s3/src/"]).unwrap();
    println!("cargo:rustc-link-arg-bins=-Tlinkall.x");
}