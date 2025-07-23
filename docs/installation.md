# インストール

## PC への Tobas のインストール

---

1. `tobas_x.x.x_amd64.deb`をダウンロードしてください．
1. Files から `tobas_x.x.x_amd64.deb`をダブルクリックすると，App Center が起動します．これには数分かかることがあります．
1. Install を押すとインストールが開始され，Applications に Tobas と Tobas Install が追加されます．これには数分かかることがあります．
1. Applications から Tobas Install を起動すると，ターミナルが開き追加のインストールが開始します．これには数十分かかることがあります．
   ターミナルの最後に`Installation finished successfully.`と表示されればインストール成功です．ターミナルを閉じてください．

![app_center](resources/installation/app_center.png)

## フライトコントローラのセットアップ

---

### 必要なもの

- <a href=https://www.raspberrypi.com/products/raspberry-pi-5/ target="_blank">Raspberry Pi 5</a>
- Tobas HAT
- 32GB 以上マイクロ SD カード (例: <a href=https://shop.sandisk.com/ja-jp/products/memory-cards/microsd-cards/sandisk-extreme-uhs-i-microsd target="_blank">SanDisk Extreme 32GB</a>)

### 手順

1. PC 側と同じバージョンの`tobas_x.x.x_arm64.img.gz`をダウンロードしてください．
1. マイクロ SD カードを，適当な SD カードリーダーを介して PC に接続してください．
1. <a href="https://etcher.balena.io/" target="_blank">balenaEtcher</a>などのイメージフラッシャーをインストールして起動してください．
1. イメージファイルにダウンロードした img を選択し，ターゲットにマイクロ SD カードを選択し，書き込みを開始してください．
1. 正常に終了したらマイクロ SD カードを PC から取り外し，ラズパイに挿入してください．
