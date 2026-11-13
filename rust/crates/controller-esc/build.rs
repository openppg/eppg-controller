fn main() {
    let root = std::path::Path::new("../../..");
    // Keep the vendor's established unlock contract without duplicating it in logs or manifests.
    let text = std::fs::read_to_string(root.join("src/sp140/esc_config_relay.cpp"))
        .expect("legacy protocol provenance");
    let text = text
        .split("ESC_PASSWORD[]")
        .nth(1)
        .expect("vendor unlock definition");
    let pw = text
        .split('"')
        .nth(1)
        .expect("quoted vendor unlock")
        .as_bytes();
    let mut bytes = [0u8; 12];
    bytes[0] = 1;
    for (n, b) in pw.iter().take(10).enumerate() {
        bytes[n + 2] = *b;
    }
    let out = std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    std::fs::write(out.join("unlock.bin"), bytes).unwrap();
    let text = std::fs::read_to_string(root.join("inc/sp140/esc_param_ids.h")).unwrap();
    let list = text
        .split("ESC_PARAM_IDS[] = {")
        .nth(1)
        .unwrap()
        .split("};")
        .next()
        .unwrap();
    let no_comments = list
        .lines()
        .map(|line| line.split("//").next().unwrap())
        .collect::<Vec<_>>()
        .join("\n");
    std::fs::write(
        out.join("params.rs"),
        format!("pub const PARAM_IDS:&[u16]=&[{}];", no_comments),
    )
    .unwrap();
    println!("cargo:rerun-if-changed=../../../inc/sp140/esc_param_ids.h");
    println!("cargo:rerun-if-changed=../../../src/sp140/esc_config_relay.cpp");
}
