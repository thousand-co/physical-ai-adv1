#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd "$(dirname "$0")" && pwd)"
case "${DELIVERY_MODE:-physical}" in
  physical) exec "$project_dir/run_physical_demo.sh" "$@" ;;
  legacy) exec "$project_dir/run_legacy_demo.sh" "$@" ;;
  *) echo "DELIVERY_MODE must be physical or legacy" >&2; exit 2 ;;
esac
