//! Serialize the legacy diagnostics response with at most one record tree live.
use serde_json::{json, Value};
use std::io::{self, Write};
fn u32at(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(b[at..at + 4].try_into().unwrap())
}
fn reason(r: u8) -> &'static str {
    match r {
        1 => "POWERON",
        2 => "EXTERNAL",
        3 => "SOFTWARE",
        4 => "PANIC",
        5 => "INT_WDT",
        6 => "TASK_WDT",
        7 => "WDT",
        8 => "DEEPSLEEP",
        9 => "BROWNOUT",
        10 => "SDIO",
        11 => "USB_UART",
        _ => "UNKNOWN",
    }
}
fn plan(r: u8) -> &'static str {
    match r {
        1 => "usb_command_reboot",
        2 => "ble_unbond_reboot",
        _ => "none",
    }
}
fn state(r: u8) -> &'static str {
    match r {
        1 => "ARMED",
        2 => "ARMED_CRUISING",
        _ => "DISARMED",
    }
}
fn record_json(b: &[u8]) -> Value {
    let mut cd = json!({"present":b[37]!=0,"valid":b[38]!=0});
    if b[37] != 0 {
        cd["task"] = Value::String(
            String::from_utf8_lossy(&b[40..56])
                .trim_end_matches('\0')
                .into(),
        );
        cd["pc"] = json!(u32at(b, 56));
        cd["cause"] = json!(u32at(b, 60));
        cd["vaddr"] = json!(u32at(b, 64));
        cd["bt"] = json!((0..b[39].min(6) as usize)
            .map(|i| u32at(b, 68 + i * 4))
            .collect::<Vec<_>>());
    }
    json!({"boot":u32at(b,0),"rst":reason(b[28]),"raw0":b[29],"raw1":b[30],"plan":plan(b[32]),"snap_ok":b[33]==1,"prev_state":state(b[31]),"prev_up_ms":u32at(b,4),"thr_age_ms":u32at(b,8),"ui_age_ms":u32at(b,12),"bms_age_ms":u32at(b,16),"free_heap":u32at(b,20),"min_heap":u32at(b,24),"esc_ok":b[34]==1,"bms_ok":b[35]==1,"cdmp":cd})
}

pub fn write_json(mut out: impl Write, current: &[u8; 92], history: &[[u8; 92]]) -> io::Result<()> {
    write!(out, "{{\"diag_v\":1,\"hist_n\":{},\"cur\":", history.len())?;
    serde_json::to_writer(&mut out, &record_json(current)).map_err(io::Error::other)?;
    out.write_all(b",\"hist\":[")?;
    for (i, record) in history.iter().enumerate() {
        if i != 0 {
            out.write_all(b",")?;
        }
        serde_json::to_writer(&mut out, &record_json(record)).map_err(io::Error::other)?;
    }
    out.write_all(b"]}")
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn complete_history_keeps_legacy_fields_and_coredump_limits() {
        let mut record = [0u8; 92];
        record[0..4].copy_from_slice(&123u32.to_le_bytes());
        record[28] = 4;
        record[31] = 2;
        record[32] = 1;
        record[33] = 1;
        record[37] = 1;
        record[38] = 1;
        record[39] = 255;
        record[40..44].copy_from_slice(b"main");
        for i in 0..6 {
            record[68 + i * 4..72 + i * 4].copy_from_slice(&(i as u32).to_le_bytes());
        }
        let history = [record; 16];
        let mut output = Vec::new();
        write_json(&mut output, &record, &history).unwrap();
        let packet: Value = serde_json::from_slice(&output).unwrap();
        assert_eq!(packet["diag_v"], 1);
        assert_eq!(packet["hist_n"], 16);
        assert_eq!(packet["hist"].as_array().unwrap().len(), 16);
        for item in packet["hist"]
            .as_array()
            .unwrap()
            .iter()
            .chain([&packet["cur"]])
        {
            assert_eq!(item["boot"], 123);
            assert_eq!(item["rst"], "PANIC");
            assert_eq!(item["prev_state"], "ARMED_CRUISING");
            assert_eq!(item["plan"], "usb_command_reboot");
            assert_eq!(item["snap_ok"], true);
            assert_eq!(item["cdmp"]["task"], "main");
            assert_eq!(item["cdmp"]["bt"], json!([0, 1, 2, 3, 4, 5]));
            assert_eq!(item.as_object().unwrap().len(), 16);
        }
        let mut empty = Vec::new();
        write_json(&mut empty, &[0; 92], &[]).unwrap();
        let empty: Value = serde_json::from_slice(&empty).unwrap();
        assert_eq!(empty["hist"], json!([]));
        assert_eq!(empty["cur"]["cdmp"], json!({"present":false,"valid":false}));
    }
}
