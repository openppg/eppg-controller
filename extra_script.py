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

def get_git_commit_count():
    # Monotonic build number: total commits reachable from HEAD. Climbs on
    # every release. Falls back to 0 when built outside a git checkout.
    try:
        return int(subprocess.check_output(['git', 'rev-list', '--count', 'HEAD']).decode('utf-8').strip())
    except Exception:
        return 0

git_revision = get_git_revision_short_hash()
version_build = get_git_commit_count()

env.Append(CPPDEFINES=[
    ("GIT_REV", f'\\"{git_revision}\\"'),
    ("VERSION_BUILD", version_build),
])

print(f"Current git revision: {git_revision} (build {version_build})")
