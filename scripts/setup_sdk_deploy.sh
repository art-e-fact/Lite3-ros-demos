#!/usr/bin/env bash
set -euo pipefail

sdk_deploy_dir="src/sdk_deploy"
sdk_deploy_url="https://github.com/art-e-fact/sdk_deploy.git"
sdk_deploy_branch="core-only"

if git -C "$sdk_deploy_dir" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    exit 0
fi

if [[ -e "$sdk_deploy_dir" ]]; then
    echo "$sdk_deploy_dir exists but is not a git checkout. Remove it and rerun setup." >&2
    exit 1
fi

git clone --branch "$sdk_deploy_branch" --depth 1 "$sdk_deploy_url" "$sdk_deploy_dir"