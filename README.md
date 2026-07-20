# MiniVisor
AArch64向けの小型Type1ハイパーバイザ

本リポジトリは[作って理解する仮想化技術─⁠─ ハイパーバイザを実装しながら仕組みを学ぶ（技術評論社,2025）](https://gihyo.jp/book/2025/978-4-297-15012-9)で実装するハイパーバイザを改造して Raspberry Pi 4 で動作することを目指すリポジトリです。

書籍では Ubunt24.04 に開発環境を構築していましたが、このリポジトリでは、Ubuntu26.04 を使うことを想定しています。
使用する QMENU、 u-boot、 buildroot のバージョンも上げています。
必要なツールのインストールのために最初に以下のコマンドを実行します。
   ```
   ./tools/install_tools.sh
   ```

書籍と同様の手順で QEMUの環境で動作しますが、`tools-pi4`以下のスクリプトを使うとRaspberry Pi 4実機向けの起動用SDカードイメージを作成できます。

## Raspberry Pi 4実機向けブートSDカードの作成手順

1. u-boot(実機向け、`rpi_arm64_defconfig`)をビルドします。
   ```
   ./tools-pi4/build_uboot.sh
   ```
2. Linuxカーネルとrootfs(`raspberrypi4_64_defconfig`)をビルドします。
   ```
   ./tools-pi4/build_buildroot.sh
   ```
3. MiniVisor本体(`mini.elf`)をビルドします。`cargo build-pi4`を実行すると、ビルド後に`bin-pi4/disk/mini.elf`へ自動でコピーされます。
   ```
   cargo build-pi4
   ```
4. Raspberry Pi公式のGPUファームウェア(`start4.elf`、`fixup4.dat`)、公式デバイスツリー(`bcm2711-rpi-4-b.dtb`)、Bluetooth無効化用のデバイスツリーオーバーレイ(`disable-bt.dtbo`)を取得し、`bin-pi4/disk/`・`bin-pi4/disk/overlays/`に配置します。本リポジトリではライセンスの都合上これらを配布していないため、[raspberrypi/firmware](https://github.com/raspberrypi/firmware/tree/master/boot)から取得してください。デバイスツリーは必ずこの公式版を使用してください。u-bootのソースツリーに同梱されている`bcm2711-rpi-4-b.dtb`には`disable-bt.dtbo`が要求する`bt`・`uart0_pins`・`bt_pins`の`__symbols__`ラベルが存在せず、overlayの適用に失敗します。`disable-bt.dtbo`自体は、RPi4のPL011 UART(uart0)を初期状態のBluetoothモジュール向け配線からGPIO14/15ピンヘッダへ付け替えるために必要です(MiniVisorのシリアルドライバはPL011のみに対応しているため、これがないとコンソール出力がBluetooth側の配線へ流れて見えなくなります)。
   ```
   curl -L -o bin-pi4/disk/start4.elf https://github.com/raspberrypi/firmware/raw/master/boot/start4.elf
   curl -L -o bin-pi4/disk/fixup4.dat https://github.com/raspberrypi/firmware/raw/master/boot/fixup4.dat
   curl -L -o bin-pi4/disk/bcm2711-rpi-4-b.dtb https://github.com/raspberrypi/firmware/raw/master/boot/bcm2711-rpi-4-b.dtb
   mkdir -p bin-pi4/disk/overlays
   curl -L -o bin-pi4/disk/overlays/disable-bt.dtbo https://github.com/raspberrypi/firmware/raw/master/boot/overlays/disable-bt.dtbo
   ```
5. SDカードイメージ`bin-pi4/disk.img`を作成します
   ```
   ./tools-pi4/create_sdcard.sh
   ```
   このスクリプトは`u-boot.bin`・`config.txt`・`mini.elf`・`boot.scr`・Linuxカーネル(`Image`)・rootfs(`DISK0`)を含むFAT32イメージを`bin-pi4/disk.img`に生成します(4.で配置した`bcm2711-rpi-4-b.dtb`・`start4.elf`・`fixup4.dat`・`overlays/disable-bt.dtbo`もそのまま含まれます)。
6. 作成した`bin-pi4/disk.img`を物理SDカードに書き込みます。`/dev/sdX`は環境に合わせて読み替えてください。
   ```
   sudo dd if=bin-pi4/disk.img of=/dev/sdX bs=4M status=progress conv=fsync
   ```
7. SDカードをRaspberry Pi 4に挿して起動します。


## ライセンスについて
本ソフトウェアはApache License, Version 2.0にてライセンスされています。
詳しくは[NOTICE](NOTICE)をご覧ください。
