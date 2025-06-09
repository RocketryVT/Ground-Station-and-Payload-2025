// use serde_generate::SourceInstaller;
// use serde_reflection::{Tracer, TracerConfig};
// use std::path::PathBuf;
// use Mesh::protocol::MiniData;

fn main() {
    return;
}

// fn main() -> Result<(), Box<dyn std::error::Error>> {
//     // Create a tracer configuration
//     let mut tracer = Tracer::new(TracerConfig::default());
    
//     // Trace the serialization of your types
//     tracer.trace_simple_type::<MiniData>()?;
//     let mini_data = tracer.registry().unwrap();

//     let install_dir = PathBuf::from("swift_output");
//     std::fs::create_dir_all(&install_dir)?;

//     let installer = serde_generate::swift::Installer::new(install_dir.clone());

//     installer.install_serde_runtime()?;

//     installer.install_bincode_runtime()?;

//     // Install the module with your types
//     let config = serde_generate::CodeGeneratorConfig::new("MiniData".to_string())
//         .with_encodings(vec![serde_generate::Encoding::Bincode]);
//     installer.install_module(&config, &mini_data)?;
    
//     println!("Swift files generated in {}", install_dir.display());

//     Ok(())
// }