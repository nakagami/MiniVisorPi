OUTPUT_FORMAT(elf64-aarch64)
ENTRY(main)

__ALIGN_SIZE = 0x1000;

# Unlike QEMU's `virt` machine (contiguous RAM from 0x40000000), the
# Raspberry Pi 4's low, 32-bit-addressable RAM bank is only usable up to
# roughly 0x3B400000 ("DRAM: 948 MiB" in u-boot's banner); everything from
# there up to the peripheral window (0xFC000000) is unmapped/reserved, and
# the remaining installed RAM is only reachable above 0x100000000. Loading
# this ELF's PT_LOAD segments at QEMU's 0x40400000 on real Pi4 hardware
# therefore writes to unmapped memory and silently hangs with no output at
# all. 0x04000000 (64 MiB) instead sits safely inside the low RAM bank, far
# below both u-boot's own relocated location and the DTB (typically placed
# near the top of that bank, close to 0x3B400000) and far above the
# "kernel_addr_r" (0x00080000) staging buffer the boot script loads this
# very ELF file into, so the segment copy source/destination never overlap.
__RAM_BASE   = 0x04000000;

SECTIONS {
  . = __RAM_BASE + SIZEOF_HEADERS;

  .text : ALIGN(__ALIGN_SIZE) {
    *(.text .text.*)
  }

  .data : ALIGN(__ALIGN_SIZE) {
    *(.data .data.*)
  }

  .rodata : ALIGN(__ALIGN_SIZE) {
    *(.rodata .rodata.*)
  }

  .bss : ALIGN(__ALIGN_SIZE) {
    *(.bss .bss.*)
  }

  .got : ALIGN(__ALIGN_SIZE) {
    *(.got .got.*)
  }
}
