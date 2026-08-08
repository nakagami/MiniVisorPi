#!/bin/bash

. tools-pi4/environment

# Sanity check: every PT_LOAD segment must sit inside the Pi4's low RAM bank
# (usable up to ~0x3B300000; u-boot places the DTB just below that, so keep a
# margin and require < 0x3B000000). A binary linked with the wrong script
# (e.g. qemu.ld's 0x40400000 base, or a corrupt mix of both scripts caused by
# passing two -T options to rust-lld) would land outside that bank and panic
# at boot when MiniVisor reserves its own segments.
limit=$((0x3B000000))
while read -r type off vaddr paddr filesz memsz rest; do
    [ "$type" = "LOAD" ] || continue
    if [ $((paddr + memsz)) -gt $limit ]; then
        echo "ERROR: $1 has a LOAD segment outside the Pi4 low RAM bank:" >&2
        echo "  paddr=$paddr memsz=$memsz (limit is 0x3B000000)" >&2
        echo "Rebuild with tools-pi4/build_minivisor.sh (do NOT pass -Tscripts/qemu.ld)." >&2
        exit 1
    fi
done < <(readelf -lW "$1")

cp $1 $DISK_IMG_DIR$BINARY_NAME
echo "mini.elf copied to $DISK_IMG_DIR$BINARY_NAME"
echo "Run tools-pi4/create_sdcard.sh $1 to build a full bootable SD card image."
