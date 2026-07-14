# MiniVisor
AArch64向けの小型Type1ハイパーバイザ

本リポジトリは[作って理解する仮想化技術─⁠─ ハイパーバイザを実装しながら仕組みを学ぶ（技術評論社,2025）](https://gihyo.jp/book/2025/978-4-297-15012-9)で実装するハイパーバイザを改造して Raspberry Pi 4 で動作することを目指すリポジトリです。

書籍と同様の手順で QEMUの環境で動作しますが、`tools-pi4`以下のスクリプトを使うとRaspberry Pi 4実機向けの起動用SDカードイメージを作成できます。

## Raspberry Pi 4実機向けブートSDカードの作成手順

1. u-boot(実機向け、`rpi_arm64_defconfig`)をビルドします。
   ```
   tools-pi4/build_uboot.sh
   ```
2. Linuxカーネルとrootfs(`raspberrypi4_64_defconfig`)をビルドします。
   ```
   tools-pi4/build_buildroot.sh
   ```
3. MiniVisor本体(`mini.elf`)をビルドします。`cargo build-pi4`を実行すると、ビルド後に`bin-pi4/disk/mini.elf`へ自動でコピーされます。
   ```
   cargo build-pi4
   ```
4. Raspberry Pi公式のGPUファームウェア(`start4.elf`、`fixup4.dat`)を取得し、`bin-pi4/disk/`に配置します。本リポジトリではライセンスの都合上これらを配布していないため、[raspberrypi/firmware](https://github.com/raspberrypi/firmware/tree/master/boot)から取得してください。
   ```
   curl -L -o bin-pi4/disk/start4.elf https://github.com/raspberrypi/firmware/raw/master/boot/start4.elf
   curl -L -o bin-pi4/disk/fixup4.dat https://github.com/raspberrypi/firmware/raw/master/boot/fixup4.dat
   ```
5. SDカードイメージ`bin-pi4/disk.img`を作成します
   ```
   tools-pi4/create_sdcard.sh
   ```
   このスクリプトは`u-boot.bin`・デバイスツリー(`bcm2711-rpi-4-b.dtb`)・`config.txt`・`mini.elf`・`boot.scr`・Linuxカーネル(`Image`)・rootfs(`DISK0`)を含むFAT32イメージを`bin-pi4/disk.img`に生成します(4.で配置した`start4.elf`・`fixup4.dat`もそのまま含まれます)。
6. 作成した`bin-pi4/disk.img`を物理SDカードに書き込みます。`/dev/sdX`は環境に合わせて読み替えてください。
   ```
   sudo dd if=bin-pi4/disk.img of=/dev/sdX bs=4M status=progress conv=fsync
   ```
7. SDカードをRaspberry Pi 4に挿して起動します。


## ライセンスについて
本ソフトウェアはApache License, Version 2.0にてライセンスされています。
詳しくは[NOTICE](NOTICE)をご覧ください。
