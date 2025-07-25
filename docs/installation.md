# インストール

## PC への Tobas のインストール

---

1. `tobas_x.x.x_amd64.deb`をダウンロードしてください．
1. エクスプローラから `tobas_x.x.x_amd64.deb`をダブルクリックすると，App Center が起動します．これには数分かかることがあります．
1. `Install`をクリックするとインストールが開始します．これには数分かかることがあります．
1. インストールが完了すると，アプリケーションメニューに`TobasInstall`，`TobasSetupAssistant`，`TobasGCS`が追加されます．
1. アプリケーションメニューから`TobasInstall`を起動すると，ターミナルが開き追加のインストールが開始します．これには数十分かかることがあります．
   ターミナルの最後に`Installation finished successfully.`と表示されればインストール成功です．ターミナルを閉じてください．

![app_center](resources/installation/app_center.png)

## フライトコントローラのセットアップ

---

### 必要なもの

- <a href=https://www.raspberrypi.com/products/raspberry-pi-5/ target="_blank">Raspberry Pi 5</a>
- Tobas HAT
- 32GB 以上マイクロ SD カード (例: <a href=https://shop.sandisk.com/ja-jp/products/memory-cards/microsd-cards/sandisk-extreme-uhs-i-microsd target="_blank">SanDisk Extreme 32GB</a>)

### 手順

1. PC にインストールしたものと同じバージョンの`tobas_x.x.x_arm64.img.gz`をダウンロードしてください．
1. マイクロ SD カードを，適当な SD カードリーダーを介して PC に接続してください．
1. <a href="https://etcher.balena.io/" target="_blank">balenaEtcher</a>などのイメージフラッシャーをインストールして起動してください．
1. イメージファイルにダウンロードしたイメージファイルを選択し，ターゲットにマイクロ SD カードを選択し，書き込みを開始してください．
1. 正常に終了したらマイクロ SD カードを PC から取り外し，FMU に挿入してください．

## 次の手順へ

---

これでインストールは完了です．
次は Tobas Setup Assistant を用いて最初のプロジェクトを作成します．
