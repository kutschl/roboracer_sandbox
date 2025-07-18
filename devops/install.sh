#!/usr/bin/env bash

# Pfad zur User-Bashrc
BASHRC="$HOME/.bashrc"
MARKER_START="# >>> Roboracer Sandbox >>>"
MARKER_END="# <<< Roboracer Sandbox <<<"

# 1) Absoluten Pfad zum Verzeichnis ermitteln, in dem dieses Skript liegt
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 2) Parent-Directory ist ROBORACER_SANDBOX_ROOT
ROBORACER_SANDBOX_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# 3) Der Block, den wir in .bashrc haben möchten
read -r -d '' EXTENSION_BLOCK <<EOF

$MARKER_START
export ROBORACER_SANDBOX_ROOT="$ROBORACER_SANDBOX_ROOT"
[ -f "\$ROBORACER_SANDBOX_ROOT/devops/env.sh" ] && source "\$ROBORACER_SANDBOX_ROOT/devops/env.sh"
$MARKER_END
EOF

# 4) Prüfen, ob bereits ein alter Block existiert
if grep -qF "$MARKER_START" "$BASHRC"; then
  echo "[↻] Ersetze bestehenden Roboracer-Block in .bashrc..."
  # Ersetze alle Zeilen vom START- bis zum END-Marker inkl. durch unseren neuen Block
  sed -i.bak "/$MARKER_START/,/$MARKER_END/c\\
$EXTENSION_BLOCK
" "$BASHRC"
  echo "[✔] Block ersetzt. Backup unter ~/.bashrc.bak angelegt."
else
  echo "[+] Füge neuen Roboracer-Block ans Ende von .bashrc hinzu..."
  printf '%s\n' "$EXTENSION_BLOCK" >> "$BASHRC"
  echo "[✔] Block hinzugefügt."
fi


if [ -f "$ROBORACER_SANDBOX_ROOT/devops/env.sh" ]; then
  source "$ROBORACER_SANDBOX_ROOT/devops/env.sh"
  echo "[✔] Loaded CLI definitions from env.sh"
else
  echo "[✘] env.sh not found at $ROBORACER_SANDBOX_ROOT/devops/env.sh" >&2
  exit 1
fi

echo "[>] Building the sandbox..."
rrs docker compose

echo "[>] Running the sandbox..."
rrs docker run
