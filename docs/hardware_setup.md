# 実機のセットアップ

## 実機の作成

---

設計通りに実機を作成します．

<!-- TODO: Navio2のような詳細な手順 -->
<!-- cf. https://docs.emlid.com/navio2/hardware-setup/ -->
<!-- cf. https://docs.emlid.com/navio2/ardupilot/typical-setup-schemes/ -->

<img src="../resources/hardware_setup/f450_1.png" alt="F450_1" width="49%"> <img src="../resources/hardware_setup/f450_2.png" alt="F450_2" width="49%">

## プロポの設定

---

S.BUS の信号は 8 チャンネル以上を想定しています．
RC 入力の各チャンネルの役割は以下のようになっています:

| チャンネル | 役割           | インターフェース |
| :--------- | :------------- | :--------------- |
| CH1        | ロール         | レバー           |
| CH2        | ピッチ         | レバー           |
| CH3        | スロットル     | レバー           |
| CH4        | ヨー           | レバー           |
| CH5        | 有効/無効      | 2 段階スイッチ   |
| CH6        | Kill           | 2 段階スイッチ   |
| CH7        | 飛行モード     | 3 段階スイッチ   |
| CH8        | サブ飛行モード | 2 段階スイッチ   |
| CH9-16     | GPSw           | 2 段階スイッチ   |

<br>

サブ飛行モードは特定の飛行モードにおける更に細かい飛行モード変更のためのスイッチです．
6 軸独立制御可能な機体の並進・回転モードの変更など，基本の 4 レバーではコマンドの自由度が不足する場合に使用されます．
GPSw (General Purpose Switch) はユーザが自由に使えるスイッチでです．
Setup Assistant で個数を設定することができます．

T10J の場合はチャンネル 1 からチャンネル 4 までは上の表で固定されており，
チャンネル 5 以降に対応するレバーをを AUX チャンネルで割り当てることができます．
今回は次のように設定しました．

| チャンネル | スイッチ |
| :--------- | :------- |
| CH5        | SwA      |
| CH6        | SwD      |
| CH7        | SwE      |
| CH8        | SwG      |
| CH9        | SwF      |
| CH10       | SwH      |

<br>

## Tobas プロジェクトの読み込みと書き込み

---

FMU を起動し，PC を FMU と同じネットワークに接続します．FMU のアクセスポイントに接続する場合，SSID と PSK は以下です．

```txt
  SSID: raspberry_wifi
  PSK : raspberry
```

アプリケーションメニューから`TobasGCS`を起動するか，もしくはターミナルで以下を実行します．

```bash
$ ros2 launch tobas_gcs gcs.launch.py
```

`Load Project`をクリックし，Setup Assistant で作成した`f450.TBS`をダブルクリックして読み込みます．
`Write Project`をクリックすると，プロジェクトが FMU に送信後にビルドされます．これには数分かかります．

![load_and_write](resources/hardware_setup/load_and_write.png)

## 各種設定

---

### Network Setting

WiFi の設定を行います．

1. `Read`をクリックすると，現在の設定が読み込まれます．
1. `Add`をクリックしてフィールドを追加し，接続するネットワークの SSID と PSK を入力してください．
1. `Write`をクリックすると，設定が FMU に反映されます．

![network_setting](resources/hardware_setup/network_setting.png)

### Accelerometer Calibration

加速度センサのキャリブレーションを行います．
機体を水平面上に置き，`Start`をクリックしてください．
数秒でキャリブレーションが完了します．

![accel_calibration](resources/hardware_setup/accel_calibration.png)

### Magnetometer Calibration

地磁気センサのキャリブレーションを行います．
地磁気センサは周囲の環境の影響を強く受けるため，FMU を機体に取り付けた状態で実行してください．

1. `Start`をクリックすると，地磁気の値が紫色の点で表示され始めます．
1. 点群がきれいな楕円体を描くように機体を全方位にゆっくりと回転させてください．6 面それぞれが上を向いた状態で Z 軸回りに 2 周ずつ回転させるのがおすすめです．
1. 完了したら`Finish`をクリックしてください．キャリブレーション後の白色の点群が原点周りのきれいな球を描いていれば成功です．

![mag_calibration](resources/hardware_setup/mag_calibration.png)

### Radio Calibration

ラジオ入力 (S.BUS) のキャリブレーションを行います．

1. `Start`をクリックすると，S.BUS の各チャンネルの値が表示され始めます．
1. それぞれのチャンネルについて，操作可能な範囲全体をカバーするようにレバーまたはスイッチを操作してください．
   レバーと GUI のバーの動作が反対方向の場合は，プロポ側の設定を適切に変更してください．
1. `Finish`をクリックすると，キャリブレーションを完了します．

![radio_calibration](resources/hardware_setup/radio_calibration.png)

### Rotor Test

<span style="color: red;"><strong>警告: この操作ではモータが回転します．プロペラを取り付けて実行する場合は十分に注意してください．</strong></span>

![rotor_test](resources/hardware_setup/rotor_test.png)

1. `Start`をクリックすると，全てのモータが回転できる状態になります．
1. それぞれのモータについて，レバーを動かして回転させ，RC 出力チャンネルと回転方向が正しいことを確認してください．
1. それぞれのモータについて，回転数が振動しない程度に制御ゲインを調整してください．
1. `Save`をクリックすると，制御ゲインが FMU に保存されます．
1. `Stop`をクリックすると，テストが終了します．

### Joint Test

機体が固定翼機の操舵面のような PWM 駆動関節を持つ場合は，それぞれの関節の位置指令テストを行うことができます．
今回の機体はプロペラ以外の可動関節をもたないためスキップします．

![joint_test](resources/hardware_setup/joint_test.png)

## 次の手順へ

---

これで作業は完了です．
次のステップではいよいよ機体を飛ばしてみます．
