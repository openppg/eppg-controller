use std::{
    env, fs,
    path::{Path, PathBuf},
};
fn sources(path: &Path, files: &mut Vec<PathBuf>) {
    for entry in fs::read_dir(path).unwrap() {
        let p = entry.unwrap().path();
        if p.is_dir() {
            sources(&p, files)
        } else if p.extension().is_some_and(|e| e == "c") {
            files.push(p)
        }
    }
}
fn main() {
    let repo = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("../../..");
    let components = env::var_os("EPPG_COMPONENTS")
        .map(PathBuf::from)
        .unwrap_or_else(|| repo.join("components"));
    let lvgl = components.join("lvgl");
    let config = repo.join("inc/sp140/lvgl");
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    let header = out.join("ui_bindings.h");
    fs::write(&header,"#include <lvgl.h>\nextern const lv_image_dsc_t cruise_control_340255_30;\nextern const lv_image_dsc_t energy_539741_26;\nextern const lv_image_dsc_t warning_2135850_30;\nextern const lv_font_t roboto_mono_16;\n").unwrap();
    let mut bindings = bindgen::Builder::default()
        .header(header.to_str().unwrap())
        .clang_arg(format!("-I{}", lvgl.display()))
        .clang_arg(format!("-I{}", config.display()))
        .clang_arg("-DLV_CONF_INCLUDE_SIMPLE")
        .prepend_enum_name(false)
        .layout_tests(false)
        .generate_comments(false)
        .wrap_static_fns(true)
        .wrap_static_fns_path(out.join("lvgl_inlines"))
        .allowlist_var("lv_font_.*")
        .allowlist_function("lv_.*")
        .allowlist_type("lv_.*")
        .allowlist_var("LV_.*")
        .allowlist_var(
            "cruise_control_340255_30|energy_539741_26|warning_2135850_30|roboto_mono_16",
        )
        .derive_default(true);
    let mut cc = cc::Build::new();
    cc.include(&lvgl)
        .include(&config)
        .define("LV_CONF_INCLUDE_SIMPLE", None)
        .define("LV_LVGL_H_INCLUDE_SIMPLE", None)
        .warnings(false);
    if env::var("TARGET").unwrap().contains("espidf") {
        let compiler = env::var("CC").unwrap_or_else(|_| "xtensa-esp32s3-elf-gcc".into());
        let output = std::process::Command::new(&compiler)
            .arg("-print-sysroot")
            .output()
            .expect("Xtensa compiler required");
        let sysroot = String::from_utf8(output.stdout).unwrap();
        bindings = bindings.clang_arg(format!("--sysroot={}", sysroot.trim()));
        cc.compiler(compiler)
            .archiver("xtensa-esp32s3-elf-ar")
            .flag("-mlongcalls");
    }
    bindings
        .generate()
        .expect("Generate exact LVGL9 bindings")
        .write_to_file(out.join("bindings.rs"))
        .unwrap();
    let mut files = Vec::new();
    sources(&lvgl.join("src"), &mut files);
    files.sort();
    cc.files(files);
    cc.file(out.join("lvgl_inlines.c"));
    for name in [
        "img/cruise-control-340255-30.c",
        "img/energy-539741-26.c",
        "img/warning_2135850_30.c",
        "fonts/roboto_mono_16.c",
    ] {
        cc.file(repo.join("src/assets").join(name));
    }
    cc.compile("controller_lvgl9");
    println!("cargo:rerun-if-env-changed=EPPG_COMPONENTS");
    println!(
        "cargo:rerun-if-changed={}",
        config.join("lv_conf.h").display()
    );
}
