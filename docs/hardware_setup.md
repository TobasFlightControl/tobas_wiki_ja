# ハードウェア設定

フライトコントローラハードウェアの設定を行います．

## 実機の作成

---

設計通りに実機を作成します．
下の画像では GNSS アンテナとプロペラはついていません．
また，フライトコントローラと地上基地局の通信は WiFi で行うため，テレメトリモジュールは不要です．

<!-- Navio2のような詳細な手順 -->
<!-- cf. https://docs.emlid.com/navio2/hardware-setup/ -->
<!-- cf. https://docs.emlid.com/navio2/ardupilot/typical-setup-schemes/ -->

<img src="../resources/hardware_setup/f450_1.png" alt="F450_1" width="49%"> <img src="../resources/hardware_setup/f450_2.png" alt="F450_2" width="49%">

実機を組み立てる際は以下の点に注意してください:

- モータの回転方向が Setup Assistant の設定と一致している
- ESC のピン番号が Setup Assistant の設定と一致している
- フライトコントローラに物理的な振動対策を施す

## プロポの設定

---

S.BUS の信号は 8 チャンネルを想定しています．
RC 入力の各チャンネルの意味は以下のようになっています:

| チャンネル | 意味       |
| :--------- | :--------- |
| CH1        | ロール     |
| CH2        | ピッチ     |
| CH3        | ヨー       |
| CH4        | スラスト   |
| CH5        | 飛行モード |
| CH6        | ---        |
| CH7        | 緊急停止   |
| CH8        | GPSw       |

GPSw (General Purpose Switch) は一般用途に使えるスイッチであり，
非平面ロータ配置マルチコプターの飛行モードの切り替えなど，制御器によっては使用することがあります．

T10J の場合はチャンネル 1 からチャンネル 4 までは上の表で固定されており，
チャンネル 5 以降に対応するレバーをを AUX チャンネルで割り当てることができます．
今回は次のように設定しました．

![aux_channel](resources/hardware_setup/aux_channel.png)

## Tobas パッケージのロードと書き込み

---

1. フライトコントローラを起動します．
1. PC をフライトコントローラと同じネットワークに接続します．
1. Applications から Tobas を起動します．
1. ヘッダの`Browse`ボタンから，Setup Assistant で作成した`tobas_f450.TBS`を選択し，`Load`ボタンで読み込みます．
1. `Write`ボタンを押して設定をラズパイに書き込みます．これには数分かかります．

## 各種設定とキャリブレーション

---

`Hardware Setup`の各タブについて，記載されている指示に従って設定を行ってください．
