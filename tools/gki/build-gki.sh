#!/bin/bash
#
# SPDX-FileCopyrightText: The LineageOS Project
# SPDX-License-Identifier: Apache-2.0
#
# Reproducible build of the GKI kernel the device ships with
# (android13-5.15, ci.android.com build 14024729, kernel/common 051a97cf151a).
#
# The result matches the stock Image byte for byte except the module signing
# certificate (a random key generated per build) and the build-id note.
#
# usage: build-gki.sh <work dir> [--verify-only]
#
#   <work dir>     repo checkout of the kernel manifest (created if missing)
#   --verify-only  skip sync and build, only check an existing build
#
# Environment overrides, for building a newer android13-5.15 release:
#   GKI_BRANCH, GKI_MANIFEST, GKI_BUILD_NUMBER, GKI_SCMVERSION, JOBS

set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)

GKI_BRANCH=${GKI_BRANCH:-common-android13-5.15-2025-07}
GKI_MANIFEST=${GKI_MANIFEST:-$HERE/manifest_14024729.xml}
GKI_BUILD_NUMBER=${GKI_BUILD_NUMBER:-14024729}
# setlocalversion needs the release tags for the -NNNNN part, which a shallow
# checkout does not have, so the full local version is given here.
GKI_SCMVERSION=${GKI_SCMVERSION:--android13-8-00044-g051a97cf151a}
JOBS=${JOBS:-$(nproc)}
STOCK_SYMBOLS=${STOCK_SYMBOLS:-$HERE/stock-module-symbols.txt}

# The embedded config (CONFIG_UNUSED_KSYMS_WHITELIST) carries the absolute
# build path, so build where Google's builder does.
BUILD_PATH=/mnt/disks/build-disk/src/android/$GKI_BRANCH

WORK=${1:?usage: build-gki.sh <work dir> [--verify-only]}
VERIFY_ONLY=${2:-}

mkdir -p "$WORK"
WORK=$(cd "$WORK" && pwd)
DIST=$WORK/out/android13-5.15/dist

if [ "$VERIFY_ONLY" != "--verify-only" ]; then
    cd "$WORK"
    if [ ! -d .repo ]; then
        repo init -q -u https://android.googlesource.com/kernel/manifest \
            -b "$GKI_BRANCH" --depth=1
    fi
    cp "$GKI_MANIFEST" .repo/manifests/
    repo init -q -m "$(basename "$GKI_MANIFEST")"
    repo sync -q -c -j8 --no-tags --no-clone-bundle --optimized-fetch

    # File modes end up in the embedded kernel headers archive.
    chmod -R go-w common build
    printf '%s\n' "$GKI_SCMVERSION" > common/.scmversion

    rm -rf out
    unshare -Urm sh -c '
        set -e
        mount -t tmpfs none /mnt
        mkdir -p "$1"
        mount --bind "$2" "$1"
        cd "$1"
        umask 022
        env -i HOME="$HOME" USER="$USER" TERM=dumb PATH=/usr/local/bin:/usr/bin:/bin \
            TZ=UTC LC_ALL=C LANG=C BUILD_NUMBER="$3" \
            BUILD_CONFIG=common/build.config.gki.aarch64 \
            build/build.sh -j"$4"
    ' sh "$BUILD_PATH" "$WORK" "$GKI_BUILD_NUMBER" "$JOBS"
fi

echo "== checking $DIST"
for f in Image Image.lz4 vmlinux.symvers System.map; do
    [ -f "$DIST/$f" ] || { echo "missing $DIST/$f"; exit 1; }
done

grep -q 'based on r450784e' "$WORK/out/android13-5.15/common/.config" ||
    { echo "not built with clang r450784e"; exit 1; }

grep -a -o -m1 'Linux version [0-9][^#]*#[^)]*UTC [0-9]*' "$DIST/Image"

# Every symbol the stock vendor modules import must exist with the same CRC.
python3 -I - "$DIST/vmlinux.symvers" "$STOCK_SYMBOLS" <<'EOF'
import sys
symvers = {}
for line in open(sys.argv[1]):
    crc, name = line.split('\t')[:2]
    symvers[name] = int(crc, 16)
missing, mismatch, ok = [], [], 0
for line in open(sys.argv[2]):
    crc, name = line.split()[:2]
    if name not in symvers:
        missing.append(name)
    elif symvers[name] != int(crc, 16):
        mismatch.append(name)
    else:
        ok += 1
print('stock module symbols: %d ok, %d missing, %d CRC mismatches'
      % (ok, len(missing), len(mismatch)))
for name in (missing + mismatch)[:20]:
    print('  ' + name)
sys.exit(1 if missing or mismatch else 0)
EOF

sha256sum "$DIST/Image.lz4"
