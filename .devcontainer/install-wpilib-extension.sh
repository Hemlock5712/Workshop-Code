#!/usr/bin/env bash
# Install the WPILib VS Code extension from the .vsix baked into the image.
# The 2027 alpha builds are published on GitHub, not on the Marketplace, so
# devcontainer.json cannot list it by ID. Runs on every attach; it is a no-op
# once the extension is in.
set -u

VSIX=/opt/wpilib/vscode-wpilib.vsix

# The base image ships a /usr/local/bin/code shim that only works once VS Code
# is attached, so probe it rather than trusting that it exists.
code_cli=""
for candidate in "$(command -v code || true)" \
  "$(ls -d "$HOME"/.vscode-remote/bin/*/bin/remote-cli/code 2>/dev/null | head -n 1)"; do
  if [ -n "$candidate" ] && "$candidate" --version >/dev/null 2>&1; then
    code_cli="$candidate"
    break
  fi
done

if [ -z "$code_cli" ]; then
  echo "WPILib extension: VS Code is not attached, skipping. To install it by hand:"
  echo "  Extensions view > ... > Install from VSIX > $VSIX"
  exit 0
fi

if "$code_cli" --list-extensions 2>/dev/null | grep -qi '^wpilibsuite.vscode-wpilib$'; then
  exit 0
fi

"$code_cli" --install-extension "$VSIX" || echo "WPILib extension: install failed, see above"
