# MiniVisor
AArch64向けの小型Type1ハイパーバイザ

本リポジトリは[作って理解する仮想化技術─⁠─ ハイパーバイザを実装しながら仕組みを学ぶ（技術評論社,2025）](https://gihyo.jp/book/2025/978-4-297-15012-9)で実装するハイパーバイザの公開リポジトリです。

## Issueについて
書籍やMiniVisorの問題については、本リポジトリのIssueに使用している環境の情報を合わせて報告していただけると幸いです。

開発環境は書籍で指定している環境以外はサポートしておりません。
指定している環境以外での問題を報告されても、対応はできませんのでご了承ください。

また、本リポジトリでは新規機能追加や改良等は行いません。
新規機能追加や改良は本リポジトリをフォークして行ってください。

## Pull Requestについて
本リポジトリでは原則としてPull Requestを受け付けていません。
書籍や本リポジトリの内容に問題がある場合は、Issueとしてご指摘ください。
ご指摘いただいた内容を元に、書籍への影響などを考慮して修正を行います。

## Raspberry Pi 4実機向けブートSDカードの作成手順
書籍で指定している環境はQEMUですが、`tools-pi4`以下のスクリプトを使うとRaspberry Pi 4実機向けの起動用SDカードイメージを作成できます。

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
4. SDカードイメージ`bin-pi4/disk.img`を作成します(3.で`bin-pi4/disk/mini.elf`にコピー済みのものを使うため、引数は不要です)。
   ```
   tools-pi4/create_sdcard.sh
   ```
   このスクリプトは`u-boot.bin`・デバイスツリー(`bcm2711-rpi-4-b.dtb`)・`config.txt`・`mini.elf`・`boot.scr`・Linuxカーネル(`Image`)・rootfs(`DISK0`)を含むFAT32イメージを`bin-pi4/disk.img`に生成します。
5. Raspberry Pi公式のGPUファームウェア(`start4.elf`、`fixup4.dat`)を追加します。本リポジトリではライセンスの都合上これらを配布していないため、[raspberrypi/firmware](https://github.com/raspberrypi/firmware/tree/master/boot)や既存のRaspberry Pi OSの`/boot/firmware`から入手し、`bin-pi4/disk/`(4.を実行する前)またはSDカードの起動パーティション(書き込み後)に配置してください。
6. 作成した`bin-pi4/disk.img`を物理SDカードに書き込みます。`/dev/sdX`は環境に合わせて読み替えてください。
   ```
   sudo dd if=bin-pi4/disk.img of=/dev/sdX bs=4M status=progress conv=fsync
   ```
7. SDカードをRaspberry Pi 4に挿して起動します。

Raspberry Pi 4実機向けの環境は書籍で指定している環境ではないため、Issueでの対応対象外です。あらかじめご了承ください。

## ライセンスについて
本ソフトウェアはApache License, Version 2.0にてライセンスされています。
詳しくは[NOTICE](NOTICE)をご覧ください。
