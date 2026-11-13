fn main() {
    embuild::espidf::sysenv::output();
    let output = std::process::Command::new("git")
        .args(["rev-list", "--count", "HEAD"])
        .output()
        .expect("git build metadata");
    assert!(output.status.success(), "git commit count unavailable");
    let build = std::str::from_utf8(&output.stdout)
        .unwrap()
        .trim()
        .parse::<u32>()
        .unwrap();
    let mut marker = *b"OPPGFWV1\x08\x01\0\0\0\0";
    marker[10..14].copy_from_slice(&build.to_le_bytes());
    let out = std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    std::fs::write(out.join("fw_marker.rs"),format!("#[used] #[no_mangle] #[link_section=\".rodata.fw_version_marker\"] pub static fw_version_marker:[u8;14]={marker:?};\npub const VERSION_BUILD:u32={build};\n")).unwrap();
    println!("cargo:rustc-link-arg=-Wl,-u,fw_version_marker");
    for args in [
        vec!["rev-parse", "--git-path", "HEAD"],
        vec!["rev-parse", "--git-path", "refs/heads/codex/rust-port"],
        vec!["rev-parse", "--git-path", "packed-refs"],
    ] {
        if let Ok(path) = std::process::Command::new("git").args(args).output() {
            if path.status.success() {
                println!(
                    "cargo:rerun-if-changed={}",
                    String::from_utf8_lossy(&path.stdout).trim()
                );
            }
        }
    }
}
