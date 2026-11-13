fn main() {
    let components = std::env::var_os("EPPG_COMPONENTS")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::path::PathBuf::from("../../../components"));
    let simulator = components.parent().unwrap().join("simulator");
    let staging = components.join("image_staging");
    cc::Build::new()
        .file(staging.join("image_staging.c"))
        .include(&staging)
        .compile("eppg_host_image_staging");
    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .file("native/nor_staging.cpp")
        .file(simulator.join("devices/peripherals.cpp"))
        .include(&staging)
        .include(&simulator)
        .compile("eppg_host_nor_staging");
    println!("cargo:rerun-if-changed=native/nor_staging.cpp");
    println!("cargo:rerun-if-changed={}", staging.display());
    println!(
        "cargo:rerun-if-changed={}",
        simulator.join("devices").display()
    );
    println!("cargo:rerun-if-env-changed=EPPG_COMPONENTS");
}
