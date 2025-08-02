import subprocess
import os
import glob

Import("env")
folder = env.GetProjectOption("src_folder")

# Generic
env.Replace(
    PROJECT_SRC_DIR="$PROJECT_DIR/src/" + folder
)

# Remove problematic LVGL files for ESP32
def remove_incompatible_files():
    try:
        # Find and remove ARM-specific assembly files (Helium and NEON)
        arm_assembly_patterns = [
            ".pio/libdeps/*/lvgl/src/draw/sw/blend/helium/*.S",
            ".pio/libdeps/*/lvgl/src/draw/sw/blend/helium/*.s",
            ".pio/libdeps/*/lvgl/src/draw/sw/blend/neon/*.S",
            ".pio/libdeps/*/lvgl/src/draw/sw/blend/neon/*.s"
        ]

        # Remove TFT_eSPI driver files (we use ST7789 instead)
        tft_espi_patterns = [
            ".pio/libdeps/*/lvgl/src/drivers/display/tft_espi/*.cpp",
            ".pio/libdeps/*/lvgl/src/drivers/display/tft_espi/*.c"
        ]

        all_patterns = arm_assembly_patterns + tft_espi_patterns

        for pattern in all_patterns:
            for file_path in glob.glob(pattern):
                if os.path.exists(file_path):
                    print(f"Removing incompatible file: {file_path}")
                    os.remove(file_path)
    except Exception as e:
        print(f"Warning: Could not remove incompatible files: {e}")

# Call the function to remove incompatible files
remove_incompatible_files()

def get_git_revision_short_hash():
    try:
        return subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).decode('utf-8').strip()
    except Exception:
        return "unknown"

git_revision = get_git_revision_short_hash()

env.Append(CPPDEFINES=[
    ("GIT_REV", f'\\"{git_revision}\\"')
])

print(f"Current git revision: {git_revision}")
