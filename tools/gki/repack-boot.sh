#!/bin/bash
#
# SPDX-FileCopyrightText: The LineageOS Project
# SPDX-License-Identifier: Apache-2.0
#
# Put a kernel into the stock boot.img layout (header v4, kernel only).
#
# usage: repack-boot.sh <gki work dir> <stock boot.img> <Image.lz4> <out boot.img>
#
# The stock image ends with Google's GKI certificate and a Motorola-signed AVB
# footer. Neither can be regenerated for a new kernel, so the output has no
# GKI certificate and an unsigned hash footer that keeps the stock footer's
# os_version, fingerprint and security_patch props. It boots only while
# vbmeta has verification disabled.

set -euo pipefail

WORK=${1:?usage: repack-boot.sh <gki work dir> <stock boot.img> <Image.lz4> <out boot.img>}
STOCK=$(readlink -f "${2:?}")
KERNEL=$(readlink -f "${3:?}")
OUT=$(readlink -f "${4:?}")

MKBOOTIMG=$WORK/tools/mkbootimg/mkbootimg.py
UNPACK=$WORK/tools/mkbootimg/unpack_bootimg.py
AVBTOOL=$WORK/prebuilts/kernel-build-tools/linux-x86/bin/avbtool

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
cd "$TMP"

python3 "$UNPACK" --boot_img "$STOCK" --out stock --format mkbootimg > args
grep -q -- '--header_version 4' args || { echo "not a v4 boot image"; exit 1; }

# The unchanged repack must reproduce the stock image up to its trailer.
eval "python3 $MKBOOTIMG $(cat args) -o same.img"
cmp -n "$(stat -c %s same.img)" "$STOCK" same.img ||
    { echo "unchanged repack differs from stock"; exit 1; }

mkdir new
cp stock/ramdisk new/ramdisk
cp "$KERNEL" new/kernel
eval "python3 $MKBOOTIMG $(sed 's#stock/#new/#g' args) -o new.img"

INFO=$("$AVBTOOL" info_image --image "$STOCK")
prop() { echo "$INFO" | sed -n "s/.*Prop: com.android.build.boot.$1 -> '\(.*\)'/\1/p"; }
SIZE=$(echo "$INFO" | awk '/^Image size:/ {print $3}')
# Reuse the stock salt so the output depends only on the kernel.
SALT=$(echo "$INFO" | awk '/Salt:/ {print $2; exit}')

"$AVBTOOL" add_hash_footer --image new.img --partition_size "$SIZE" \
    --partition_name boot --hash_algorithm sha256 --algorithm NONE --salt "$SALT" \
    --prop "com.android.build.boot.os_version:$(prop os_version)" \
    --prop "com.android.build.boot.fingerprint:$(prop fingerprint)" \
    --prop "com.android.build.boot.security_patch:$(prop security_patch)"

# Apart from the kernel, the headers of both images must be the same.
python3 "$UNPACK" --boot_img "$STOCK" --out v-stock | grep -v '^kernel_size' > h-stock
python3 "$UNPACK" --boot_img new.img --out v-new | grep -v '^kernel_size' > h-new
diff h-stock h-new || { echo "header differs beyond the kernel"; exit 1; }
cmp -s v-stock/ramdisk v-new/ramdisk || { echo "ramdisk differs"; exit 1; }
cmp -s v-new/kernel "$KERNEL" || { echo "kernel not packed as given"; exit 1; }

cp new.img "$OUT"
sha256sum "$OUT"
