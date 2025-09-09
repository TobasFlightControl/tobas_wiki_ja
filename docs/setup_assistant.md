# Tobas Setup Assistant

<!-- ゲームの広告と同じで，全てを理解することよりもとりあえず何も考えずに簡単に動かせることが大事． -->
<!-- 後々必要になる面倒な作業は隠して面白いところを見せる． -->

Tobas Setup Assistant は，Tobas を用いてドローンを飛ばすのに必要なプロジェクトフォルダを作成するための GUI です．
プロジェクトフォルダには，機体の質量特性，プロペラの空気特性，モータの電気特性といった，ドローンを飛ばすのに必要な全ての情報が含まれます．
Tobas Setup Assistant を使用するためには，ユーザの機体を表現した Universal Aircraft Description Language (UADF) ファイルが必要です．
UADF の詳細については[What is UADF](./what_is_uadf.md)をご参照ください．

## 準備

---

このチュートリアルでは，<a href=https://www-v1.dji.com/jp/flame-wheel-arf/feature.html target="_blank">DJI F450</a>を使用します．
機体の UADF を作る必要がありますが，今回は既に用意してあるものを使用します．

## 起動

---

アプリケーションメニューから`TobasSetupAssistant`を起動するか，もしくはターミナルで以下を実行します．

```bash
$ ros2 launch tobas_setup_assistant setup_assistant.launch.py
```

![start](resources/setup_assistant/start.png)

## UADF の読み込み

---

`New`をクリックし，ファイルダイアログで`/opt/tobas/share/tobas_description/urdf/f450.uadf`を選択して`Open`をクリックします．
すると，モデルビューに機体が表示され，各設定ページが有効になります．

![load](resources/setup_assistant/load.png)

## Propulsion System

---

推進系の設定を行います．
今回は電動機のため`Electric Propulsion System`にチェックを入れたままにします．

### Battery

バッテリーの設定を行います．
バッテリーの仕様を確認し，各項目に適切な値を入力してください．

![battery](resources/setup_assistant/propulsion/battery.png)

### Propulsion Units

各推進ユニットの設定を行います．

まず`propeller_0`リンクの設定を行います．
機体との対応関係がわからない場合は，画面左上の`Frame Tree`からリンク名をクリックすることでモデルビューにハイライトされます．
各部品の仕様を確認し，`ESC`，`Motor`，`Propeller`の各項目に適切な値を入力してください．

![propulsion/esc](resources/setup_assistant/propulsion/esc.png)

![propulsion/motor](resources/setup_assistant/propulsion/motor.png)

![propulsion/propeller](resources/setup_assistant/propulsion/propeller.png)

`Aerodynamics`ではプロペラの空力特性の設定を行います．
複数の設定方法から選ぶことができますが，プロペラ単体試験のデータを使用するのが望ましいです．
今回はプルダウンから`Estimate from Thrust Stand Data`を選択します．
`Load CSV`をクリックし，ファイルダイアログで`/opt/tobas/share/tobas_setup_assistant/data/thrust_stand/dji_9450.csv`を選択して`Open`をクリックします．
すると，プロペラの試験データがテーブルに反映されます．

![propulsion/aerodynamics](resources/setup_assistant/propulsion/aerodynamics.png)

この機体の 4 つの推進モジュールは全て同一なため，`Copy To All`をクリックし，`propeller_0`の設定を他の 3 つにコピーします．
`propeller_0`の設定が他のタブにも反映されていることを確認してください．

## Flight Management Unit

---

ハードウェアに関するを設定を行います．
`Tobas T1`が選択されていることを確認し，
4 つの推進ユニットそれぞれについて，適切に DShot チャンネルを設定してください．

![fmu](resources/setup_assistant/fmu.png)

## プロジェクトの保存

`Save`をクリックし，ファイルダイアログで`~/Tobas/colcon_ws/src/`以下に`f450.TBS`として保存します．

![save](resources/setup_assistant/save.png)

## 次の手順へ

---

これで作業は完了です．
Setup Assistant を閉じてください．
次は，実機を作成してハードウェアのセットアップを行います．
