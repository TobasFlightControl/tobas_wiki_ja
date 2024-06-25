# ハードウェア設定

フライトコントローラハードウェアの設定を行います．
`Radio Calibration`以外はシミュレーションの後でも構いません．

## 実機の作成

---

以下のサイトを参考に，実機を作成します．

- <a href=https://docs.emlid.com/navio2/hardware-setup target="_blank">Hardware setup | Navio2</a>
- <a href=https://docs.emlid.com/navio2/ardupilot/typical-setup-schemes target="_blank">Typical setup schemes | Navio2</a>

<img src="../resources/hardware_setup/f450_1.png" alt="F450_1" width="49%"> <img src="../resources/hardware_setup/f450_2.png" alt="F450_2" width="49%">

その際に以下の点に注意してください:

- モータの回転方向が Setup Assistant の設定と一致している
- ESC のピン番号が Setup Assistant の設定と一致している
- フライトコントローラに物理的な振動対策を施す

また，フライトコントローラと地上基地局の通信は WiFi で行うため，テレメトリモジュールは不要です．

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

1. ラズパイのアクセスポイントに接続します．SSID は`raspberry_wifi`，パスワードは`raspberry`です．
1. Applications から Tobas を起動します．
1. ヘッダーの`Load`ボタンを押し，Setup Assistant で作成した`tobas_f450.TBS`を選択します．
1. `Send`ボタンを押して設定をラズパイに書き込みます．これには数分かかります．

## 各種設定とキャリブレーション

---

`Hardware Setup`の各タブについて，記載されている指示に従って設定を行ってください．
