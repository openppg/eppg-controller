fn main() {
    let source = std::env::var_os("EPPG_COMPONENTS")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::path::PathBuf::from("../../../components"))
        .join("dronecan/src");
    let mut build = cc::Build::new();
    if std::env::var("TARGET").unwrap().contains("xtensa-esp32s3") {
        // cc's generic Xtensa clang selection does not establish the S3 little-endian ABI.
        // IDF firmware uses static code; cc otherwise enables PIC for espidf.
        build
            .pic(false)
            .compiler("xtensa-esp32s3-elf-gcc")
            .archiver("xtensa-esp32s3-elf-ar")
            .flag("-mlongcalls");
    }
    build
        .file("c/bridge.c")
        .file(source.join("canard.c"))
        .include(&source)
        .define("CANARD_ENABLE_CANFD", "0")
        .warnings(false)
        .compile("eppg_dronecan");
    println!("cargo:rerun-if-changed=c/bridge.c");
    println!("cargo:rerun-if-changed={}", source.display());
    println!("cargo:rerun-if-env-changed=EPPG_COMPONENTS");
}
