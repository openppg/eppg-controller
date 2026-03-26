import os
import subprocess

Import("env")
folder = env.GetProjectOption("custom_src_folder")

# Generic
env.Replace(
    PROJECT_SRC_DIR="$PROJECT_DIR/src/" + folder
)

def get_git_revision_short_hash():
    try:
        return subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).decode('utf-8').strip()
    except Exception:
        return "unknown"

def parse_build_number(candidate):
    if candidate is None:
        return None

    candidate = str(candidate).strip()
    if not candidate.isdigit():
        return None

    return int(candidate)

def get_build_number():
    for candidate in (
        os.environ.get("OPENPPG_FIRMWARE_BUILD"),
        os.environ.get("GITHUB_RUN_NUMBER"),
    ):
        parsed = parse_build_number(candidate)
        if parsed is not None:
            return parsed

    try:
        return int(
            subprocess.check_output(['git', 'rev-list', '--count', 'HEAD']).decode('utf-8').strip()
        )
    except Exception:
        return 0

git_revision = get_git_revision_short_hash()
build_number = get_build_number()

env.Append(CPPDEFINES=[
    ("GIT_REV", f'\\"{git_revision}\\"'),
    ("VERSION_BUILD", build_number),
])

print(f"Current git revision: {git_revision}")
print(f"Firmware build number: {build_number}")
