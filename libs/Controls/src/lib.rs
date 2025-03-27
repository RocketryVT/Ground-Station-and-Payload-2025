#![no_std]
#![no_main]

pub mod pid;
pub mod high_pass;
pub mod low_pass;
pub mod ekf;

// Re-export the madgwick module/crate
// ahrs actually has madgwick and mahony filters based on the new Fusion library
pub use ahrs as madgwick;