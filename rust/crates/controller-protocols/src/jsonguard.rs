//! Bounds JSON nesting before a recursive parser sees untrusted input.
/// Deepest container nesting any controller message uses, with margin.
pub const MAX_DEPTH: usize = 6;
/// True when `line` never nests objects/arrays deeper than `max`. Brackets inside string
/// literals are ignored. A recursive JSON parser uses stack per level, so a line of a few
/// hundred `[` bytes would otherwise overflow a small task stack.
pub fn depth_within(line: &[u8], max: usize) -> bool {
    let (mut depth, mut in_string, mut escaped) = (0usize, false, false);
    for &byte in line {
        if in_string {
            if escaped {
                escaped = false;
            } else if byte == b'\\' {
                escaped = true;
            } else if byte == b'"' {
                in_string = false;
            }
            continue;
        }
        match byte {
            b'"' => in_string = true,
            b'{' | b'[' => {
                depth += 1;
                if depth > max {
                    return false;
                }
            }
            b'}' | b']' => depth = depth.saturating_sub(1),
            _ => {}
        }
    }
    true
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn real_messages_pass_and_deep_nesting_is_refused() {
        assert!(depth_within(br#"{"command":"sync"}"#, MAX_DEPTH));
        assert!(depth_within(
            br#"{"settings":{"theme":1,"screen_rot":3},"wireless":{"role":"disabled"}}"#,
            MAX_DEPTH
        ));
        assert!(depth_within(
            br#"{"a":"[[[[[[[[[[[[[[[[ \" [[[[[[["}"#,
            MAX_DEPTH
        ));
        assert!(!depth_within(&[b'['; 250], MAX_DEPTH));
        assert!(!depth_within(
            br#"{"a":{"b":{"c":{"d":{"e":{"f":{"g":1}}}}}}}"#,
            MAX_DEPTH
        ));
        assert!(depth_within(b"[][][][][][][][][][][]", MAX_DEPTH));
        assert!(depth_within(b"]]]]]]{}", MAX_DEPTH));
    }
}
