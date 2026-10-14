#!/usr/bin/env bash
# Run the automated RC checks for the committed HEAD on workbox and copy
# rc-results.json back to .rc-results/ here. From Git Bash on PDPC-MAIN:
#
#   bash tools/rc_test/run_on_workbox.sh
#
# Then drop .rc-results/rc-results.json onto the RC checklist page.

set -euo pipefail
ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"
SSH="${SSH:-/c/Windows/System32/OpenSSH/ssh.exe}"
SCP="${SCP:-/c/Windows/System32/OpenSSH/scp.exe}"
command -v "$SSH" >/dev/null 2>&1 || SSH=ssh
command -v "$SCP" >/dev/null 2>&1 || SCP=scp
REMOTE_DIR="projects/eppg-rc-test"

if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
  echo "Note: uncommitted changes are not tested; this runs commit $(git rev-parse --short HEAD)."
fi

BRANCH="$(git rev-parse --abbrev-ref HEAD)"
BUNDLE="$(mktemp -d)/rc.bundle"
git bundle create "$BUNDLE" HEAD >/dev/null 2>&1
"$SSH" workbox "mkdir -p ~/incoming ~/logs"
"$SCP" -q "$BUNDLE" workbox:incoming/rc-test.bundle

"$SSH" workbox "bash -s" <<EOF
set -e
cd ~
if [ ! -d $REMOTE_DIR/.git ]; then
  git clone -q ~/incoming/rc-test.bundle $REMOTE_DIR
fi
cd $REMOTE_DIR
git fetch -q ~/incoming/rc-test.bundle HEAD
git checkout -q -f FETCH_HEAD
git clean -q -fdx -e .pio -e build-screenshot
echo "Testing \$(git log -1 --format='%h %s') on workbox"
RC_BRANCH=$BRANCH systemd-run --user --scope -q -p MemoryMax=6G bash tools/rc_test/run_rc_tests.sh 2>&1 | tee ~/logs/rc-test.log
EOF

mkdir -p .rc-results
"$SCP" -q "workbox:$REMOTE_DIR/.rc-results/rc-results.json" .rc-results/rc-results.json
echo
echo "Copied to $(pwd)/.rc-results/rc-results.json"
echo "Open the RC checklist page and drop that file on it."
