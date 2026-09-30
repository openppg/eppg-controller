#!/usr/bin/env python3
"""Regenerate the hinted Montserrat fonts (src/assets/fonts/montserrat_hinted_*.c).

Same face and character set as LVGL's built-in Montserrat fonts, but with
strong autohinting, so vertical stems land on whole pixels and small text
on the 160x128 display is crisp instead of smeared across grey columns.

    npm install lv_font_conv@1.5.3          (any folder)
    python tools/fonts/gen_hinted_fonts.py --lv-font-conv path/to/lv_font_conv.js \\
        --lvgl-src path/to/lvgl                (e.g. build-screenshot/_deps/lvgl-src)

The generated files use the same file-local names (glyph_bitmap, font_dsc,
...), so each is prefixed with its font name; that lets
src/sp140/lvgl/lvgl_fonts.cpp compile them all in one translation unit.
"""

import argparse
import os
import re
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, "..", "..", "src", "assets", "fonts"))
SIZES = [10, 12, 14, 16, 18, 24, 28]
DEFAULT_SIZE = 14  # LVGL's default font: also carries the LV_SYMBOL_* icons
TEXT_RANGE = "0x20-0x7F,0xB0,0x2022"
LOCAL_NAMES = ["glyph_bitmap", "glyph_dsc", "cmaps", "kern_left_class_mapping",
               "kern_right_class_mapping", "kern_class_values", "kern_classes",
               "kern_pair_glyph_ids", "kern_pair_values", "kern_pairs", "cache",
               "font_dsc"]


def symbol_range(lvgl_src):
    """The icon code points LVGL builds into its own Montserrat 14."""
    path = os.path.join(lvgl_src, "src", "font", "lv_font_montserrat_14.c")
    head = open(path, encoding="utf-8").read(4000)
    return re.search(r"FontAwesome5-Solid\+Brands\+Regular\.woff -r ([0-9,]+)", head).group(1)


def prefix_locals(text, prefix):
    names = LOCAL_NAMES + sorted(set(re.findall(r"\bunicode_list_\d+\b", text)))
    for name in names:
        # Not after '.', so designated-initializer fields keep their names.
        text = re.sub(r"(?<![.\w])%s\b" % re.escape(name), f"{prefix}_{name}", text)
    return text


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--lv-font-conv", required=True, help="path to lv_font_conv.js")
    ap.add_argument("--lvgl-src", required=True, help="LVGL source tree (fonts + symbols)")
    args = ap.parse_args()
    font_dir = os.path.join(args.lvgl_src, "scripts", "built_in_font")
    symbols = symbol_range(args.lvgl_src)

    for size in SIZES:
        name = f"montserrat_hinted_{size}"
        out = os.path.join(OUT, f"{name}.c")
        cmd = ["node", args.lv_font_conv, "--no-compress", "--no-prefilter", "--bpp", "4",
               "--size", str(size),
               "--font", os.path.join(font_dir, "Montserrat-Medium.ttf"), "--autohint-strong",
               "-r", TEXT_RANGE]
        if size == DEFAULT_SIZE:
            cmd += ["--font", os.path.join(font_dir, "FontAwesome5-Solid+Brands+Regular.woff"),
                    "-r", symbols]
        # Run in the output folder so the provenance comment names a bare file.
        cmd += ["--format", "lvgl", "--force-fast-kern-format", "-o", f"{name}.c"]
        subprocess.run(cmd, check=True, capture_output=True, cwd=OUT)

        text = open(out, encoding="utf-8").read()
        # Keep the provenance comment free of local paths.
        text = re.sub(r"--font \S*[/\\](Montserrat-Medium\.ttf)", r"--font \1", text)
        text = re.sub(r"--font \S*[/\\](FontAwesome5-Solid\+Brands\+Regular\.woff)", r"--font \1", text)
        text = prefix_locals(text, name)
        open(out, "w", encoding="utf-8", newline="\n").write(text)
        print(f"wrote {os.path.relpath(out)}")


if __name__ == "__main__":
    main()
