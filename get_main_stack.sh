#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
STACK_DIR="$ROOT/external/IITBDV-main-stack"
REPO_URL="https://github.com/IIT-Bombay-Racing-Driverless/IITBDV-main-stack.git"

if git -C "$STACK_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  BRANCH="$(git -C "$STACK_DIR" branch --show-current)"
  if [ "$BRANCH" != "devel" ]; then
    echo "IITBDV-main-stack exists but is on branch '$BRANCH', not 'devel'." >&2
    echo "Leaving it unchanged." >&2
    exit 1
  fi
  echo "IITBDV-main-stack is already present on branch devel."
elif [ -e "$STACK_DIR" ]; then
  echo "$STACK_DIR exists but is not a Git checkout; leaving it unchanged." >&2
  exit 1
else
  mkdir -p "$ROOT/external"
  git clone --depth 1 --single-branch --branch devel \
    "$REPO_URL" "$STACK_DIR"
fi
