#!/usr/bin/env bash
# UzMap downstream fork toolchain preflight (KAN-69 artifact contract, PORTING.md §10).
# Verifies exact toolchains; prints one line per check and exits 1 if any fail.
set -uo pipefail

REQUIRED_RUST="1.97.1"
REQUIRED_CARGO_NDK="4.1.2"
REQUIRED_NDK="26.2.11394342"
REQUIRED_JDK="21"
APPLE_TARGETS=(aarch64-apple-ios aarch64-apple-ios-sim x86_64-apple-ios)
ANDROID_TARGETS=(aarch64-linux-android armv7-linux-androideabi i686-linux-android x86_64-linux-android)

failures=0
ok()   { echo "OK   $1"; }
fail() { echo "FAIL $1"; failures=$((failures + 1)); }

if rustup toolchain list 2>/dev/null | grep -q "^${REQUIRED_RUST}"; then
  ok "rust toolchain ${REQUIRED_RUST}"
  installed_targets="$(rustup target list --toolchain "${REQUIRED_RUST}" --installed 2>/dev/null)"
  for t in "${APPLE_TARGETS[@]}" "${ANDROID_TARGETS[@]}"; do
    if printf '%s\n' "$installed_targets" | grep -qx "$t"; then
      ok "rust target $t"
    else
      fail "rust target $t missing for ${REQUIRED_RUST}"
    fi
  done
else
  fail "rust toolchain ${REQUIRED_RUST} not installed"
fi

cargo_ndk_version="$(cargo ndk --version 2>/dev/null | awk '{print $NF}')"
if [[ "$cargo_ndk_version" == "$REQUIRED_CARGO_NDK" ]]; then
  ok "cargo-ndk ${REQUIRED_CARGO_NDK}"
else
  fail "cargo-ndk ${REQUIRED_CARGO_NDK} required, found '${cargo_ndk_version:-none}'"
fi

ndk_dir=""
for candidate in "${ANDROID_NDK_HOME:-}" "${ANDROID_HOME:-}/ndk/${REQUIRED_NDK}" \
  "${ANDROID_SDK_ROOT:-}/ndk/${REQUIRED_NDK}" "$HOME/Library/Android/sdk/ndk/${REQUIRED_NDK}"; do
  if [[ -n "$candidate" && -d "$candidate" ]]; then
    ndk_dir="$candidate"
    break
  fi
done
if [[ -n "$ndk_dir" ]]; then
  ok "android ndk ${REQUIRED_NDK} at $ndk_dir"
else
  fail "android ndk ${REQUIRED_NDK} not found"
fi

jdk21_home=""
is_jdk21() { [[ -x "$1/bin/java" ]] && "$1/bin/java" -version 2>&1 | head -1 | grep -q "\"${REQUIRED_JDK}\."; }
for candidate in "${JAVA_HOME:-}" /opt/homebrew/opt/openjdk@21 \
  /opt/homebrew/opt/openjdk@21/libexec/Home; do
  [[ -z "$candidate" ]] && continue
  if is_jdk21 "$candidate"; then
    jdk21_home="$candidate"
    break
  fi
done
if [[ -z "$jdk21_home" ]] && [[ -x /usr/libexec/java_home ]]; then
  v="$(/usr/libexec/java_home -v "$REQUIRED_JDK" 2>/dev/null || true)"
  if [[ -n "$v" ]] && is_jdk21 "$v"; then
    jdk21_home="$v"
  fi
fi
if [[ -n "$jdk21_home" ]]; then
  ok "jdk ${REQUIRED_JDK} at $jdk21_home"
else
  fail "jdk ${REQUIRED_JDK} not found"
fi

if command -v just >/dev/null 2>&1; then
  ok "just $(just --version | awk '{print $2}')"
else
  fail "just not installed"
fi

if [[ "$failures" -gt 0 ]]; then
  echo "PREFLIGHT_FAILED failures=$failures" >&2
  exit 1
fi
echo "PREFLIGHT_OK"
