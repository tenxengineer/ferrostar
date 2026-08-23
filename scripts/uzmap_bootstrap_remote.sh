#!/usr/bin/env bash
# UzMap downstream fork remote bootstrap (KAN-69 artifact contract, PORTING.md §10).
#
# Invariants enforced before ANY mutation:
#   - origin fetch list must be exactly [FORK_URL]            (else exit 66)
#   - origin effective-push list must be exactly [FORK_URL]   (else exit 67)
#   - upstream fetch list, if the remote exists, must be
#     exactly [UPSTREAM_URL]                                  (else exit 68)
# Mutations (idempotent):
#   - create upstream remote with UPSTREAM_URL if absent
#   - normalize upstream effective-push list to the single sentinel DISABLED
set -euo pipefail

FORK_URL="https://github.com/tenxengineer/ferrostar.git"
UPSTREAM_URL="https://github.com/stadiamaps/ferrostar.git"
PUSH_SENTINEL="DISABLED"

cd "$(git rev-parse --show-toplevel)"

origin_fetch="$(git remote get-url --all origin 2>/dev/null || true)"
if [[ "$origin_fetch" != "$FORK_URL" ]]; then
  echo "bootstrap: origin fetch list is not exactly [$FORK_URL]:" >&2
  printf '%s\n' "$origin_fetch" >&2
  exit 66
fi

origin_push="$(git remote get-url --all --push origin 2>/dev/null || true)"
if [[ "$origin_push" != "$FORK_URL" ]]; then
  echo "bootstrap: origin effective-push list is not exactly [$FORK_URL]:" >&2
  printf '%s\n' "$origin_push" >&2
  exit 67
fi

if upstream_fetch="$(git remote get-url --all upstream 2>/dev/null)"; then
  if [[ "$upstream_fetch" != "$UPSTREAM_URL" ]]; then
    echo "bootstrap: upstream fetch list is not exactly [$UPSTREAM_URL]:" >&2
    printf '%s\n' "$upstream_fetch" >&2
    exit 68
  fi
else
  git remote add upstream "$UPSTREAM_URL"
fi

upstream_push="$(git remote get-url --all --push upstream 2>/dev/null || true)"
if [[ "$upstream_push" != "$PUSH_SENTINEL" ]]; then
  git remote set-url --push upstream "$PUSH_SENTINEL"
fi

config_hash="$(git config --local --list | sha256sum | cut -d' ' -f1)"
fork_sha="$(git rev-parse HEAD)"
echo "BOOTSTRAP_OK config_sha256=$config_hash fork_sha=$fork_sha"
