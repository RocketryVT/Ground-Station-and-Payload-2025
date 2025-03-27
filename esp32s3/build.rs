fn main() {
    println!("cargo:rustc-link-arg-bins=-Tlinkall.x");
}
// fn main() {
//     // Check which feature is enabled and set the target triple accordingly
//     if cfg!(feature = "esp32s3") {
//         println!("cargo:rustc-env=TARGET_TRIPLE=xtensa-esp32s3-none-elf");
//     } else if cfg!(feature = "esp32") {
//         println!("cargo:rustc-env=TARGET_TRIPLE=xtensa-esp32-none-elf");
//     } else {
//         panic!("No valid feature enabled. Please enable either `esp32` or `esp32s3`.");
//     }
    
//     println!("cargo:rustc-link-arg-bins=-Tlinkall.x");
// }