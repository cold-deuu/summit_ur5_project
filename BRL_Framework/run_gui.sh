#!/usr/bin/env bash
set -euo pipefail
GUI_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GUI_PYTHON="${BRL_GUI_PYTHON:-/usr/bin/python3}"
if ! "$GUI_PYTHON" -c 'import PyQt5.QtWidgets, yaml' >/dev/null 2>&1; then
  echo "PyQt5 and PyYAML are required (Ubuntu: sudo apt install python3-pyqt5 python3-yaml)." >&2
  echo "Set BRL_GUI_PYTHON to select another Python interpreter." >&2
  exit 1
fi
exec "$GUI_PYTHON" "$GUI_DIR/run_master_gui.py" "$@"
