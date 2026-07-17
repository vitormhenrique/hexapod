#!/usr/bin/env sh

set -eu

workspace_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
setup_script="$workspace_dir/install/setup.sh"

if [ -f "$setup_script" ]; then
  set +u
  . "$setup_script"
  set -u
fi

exec "$@"