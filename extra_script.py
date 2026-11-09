import os
import subprocess
import sys

Import("env")

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

def is_shallow_clone():
    try:
        out = subprocess.check_output(['git', 'rev-parse', '--is-shallow-repository'])
        return out.decode('utf-8').strip() == 'true'
    except Exception:
        return False

git_revision = get_git_revision_short_hash()
version_build = get_git_commit_count()

# A shallow clone undercounts commits, so the build number would be wrong. The
# OTA server orders releases by it, so CI must never ship such a build.
if is_shallow_clone():
    message = (f"Shallow git clone: build number {version_build} is not the real "
               "commit count. Fetch full history (git fetch --unshallow).")
    if os.environ.get('GITHUB_ACTIONS') == 'true':
        sys.stderr.write(f"Error: {message}\n")
        env.Exit(1)
    print(f"WARNING: {message}")

env.Append(CPPDEFINES=[
    ("GIT_REV", f'\\"{git_revision}\\"'),
    ("VERSION_BUILD", version_build),
])

print(f"Current git revision: {git_revision} (build {version_build})")
