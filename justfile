# UzMap downstream fork orchestration (KAN-69 artifact contract).
# Toolchain pin is applied by the caller: RUSTUP_TOOLCHAIN=1.97.1 just test-common
set shell := ["bash", "-euo", "pipefail", "-c"]

# Verify/normalize fork remotes (origin=fork only, upstream push=DISABLED).
bootstrap-remote:
    ./scripts/uzmap_bootstrap_remote.sh

# Verify exact toolchains; runs remote bootstrap first per contract.
preflight: bootstrap-remote
    ./scripts/uzmap_preflight.sh

# Common core tests + doc tests (oracle gates 2 and 6).
test-common:
    cd common && cargo test -p ferrostar
    cd common && cargo test -p ferrostar --doc
