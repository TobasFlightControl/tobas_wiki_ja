# Tobas Setup Assistant

Tobas Setup Assistant は，Tobas を用いてドローンを飛ばすのに必要な設定ファイルを生成するための GUI です．
前ページで作成した URDF を読み込み，プロペラの空気力学，モータの電気力学，センサノイズなどの URDF には表現されていない項目の設定を行います．

## Setup Assistant の起動と URDF のロード

---

Tobas を起動し，ツールボタンから`Setup Assistant`を選択します．

![start](resources/setup_assistant/start.png)

`Create new Tobas configuration package`にチェックが入っていることを確認します．
`Load`ボタンを押して先程作成した URDF を選択すると，URDF がロードされます．
画面左上の`Frames Tree`にはリンク名がツリー状に表示されており，リンク名をクリックすると中央のモデルビューで対応するリンクがハイライトされます．
画面右上には全ての可動関節名が表示されており，バーを動かすと中央のモデルビューで対応する関節角が変化します．

![load](resources/setup_assistant/load.png)

ロードが完了すると画面左のタブがアクティブになります．
これらのタブを上から順に設定していきます．

## Battery

---

バッテリーの設定を行います．
バッテリーの仕様を見ながら以下のように設定します:

![battery](resources/setup_assistant/battery.png)

## Propulsion

---

推進系 (プロペラ) の設定を行います．
`Available Links`にプロペラとして利用可能なリンクが表示されています．
表示されない場合は，URDF Builder でプロペラリンクのジョイントタイプが`Continuous`になっていることを確認してください．

![propulsion/available_links](resources/setup_assistant/propulsion/available_links.png)

リンク名の右端の`Add`ボタンを押すと，モデルビューに推力方向が矢印で表示されると同時にそのプロペラの設定タブが追加されます．
推力の向きが間違っている場合は，URDF Builder でジョイントの`Axis`を修正してください．

![propulsion/selected_links](resources/setup_assistant/propulsion/selected_links.png)

`propeller_0`リンクの設定を行います．
各パーツのデータシートを見ながら，`General`，`ESC`，`Motor`，`Propeller`の各項目に適切な値を入力します．

![propulsion/general](resources/setup_assistant/propulsion/general.png)

![propulsion/esc](resources/setup_assistant/propulsion/esc.png)

![propulsion/motor](resources/setup_assistant/propulsion/motor.png)

![propulsion/propeller](resources/setup_assistant/propulsion/propeller.png)

`Aerodynamics`ではプロペラの空力特性の設定を行います．
複数の設定方法から選ぶことができ，
精度の観点から`Estimate from Thrust Stand Data`または`Estimate from UIUC Propeller Data`が望ましいです．
今回は`Estimate from UIUC Propeller Data`を選択します．
<a href=https://m-selig.ae.illinois.edu/props/propDB.html target="_blank">UIUC Propeller Data Site</a>
とは様々なプロペラの空力特性の実験データをまとめたサイトであり，
例えば<a href=https://www.apcprop.com/ target="_blank">APC</a>のプロペラならば大抵のものはデータを得ることができます．
とはいえ今回使用している Phantom3 0945 のデータはないため，UIUC の Volume 1 の
<a href=https://m-selig.ae.illinois.edu/props/volume-1/data/apcsf_9x4.7_static_kt1032.txt target="_blank">9 X 4.7 のデータ</a>
で代用します．
Static データをテーブルに転記してください．

![propulsion/aerodynamics](resources/setup_assistant/propulsion/aerodynamics.png)

以下のような CSV ファイルを作成して Load CSV からロードすることもできます:

```csv
RPM,CT,CP
2763,0.1137,0.0481
3062,0.1152,0.0482
3310,0.1152,0.0481
3622,0.1158,0.0480
3874,0.1159,0.0480
4153,0.1163,0.0480
4422,0.1170,0.0481
4687,0.1176,0.0482
4942,0.1175,0.0481
5226,0.1184,0.0484
5473,0.1189,0.0484
5736,0.1190,0.0484
6026,0.1192,0.0484
6285,0.1192,0.0483
6554,0.1195,0.0483
6768,0.1199,0.0483
```

他の 3 枚のプロペラについても同様の設定を行う必要がありますが，RC 出力チャンネルと回転方向以外は同じなのでコピーします．
タブ上部の `Copy To All` を押して`propeller_0`の設定を他の全てのプロペラリンクにコピーします．
`propeller_0`の設定が他のリンクにも反映されていることを確認してください．

各プロペラの`General`ページから`Channel`と`Turning Direction`を適切に設定します．
`Channel`は`propeller_0`から順に 0,1,2,3 としています．
リンク名と位置の対応がわからない場合は`Frames Tree`のハイライト機能を用いて確認してください．

## Fixed Wing

---

固定翼の設定を行います．
今回は回転翼機なのでパスします．

## Joint Config

---

機体がもつ全ての可動関節に関する設定を行います．
今回は全ての可動関節を推進系に用いているため，ここで編集できるフィールドはありませんが，
ティルトロータやマニピュレーション用の PWM 駆動関節を持つ場合はここで役割やインターフェース等に関する設定を行います．

![joint_config](resources/setup_assistant/joint_config.png)

## オンボードセンサ

---

加速度センサ，ジャイロセンサ，地磁気センサ，気圧センサ，GNSS レシーバはフライトコントローラに組み込まれています．
基本的に設定はデフォルトで構いませんが，センサ位置がルートフレームから離れている場合はオフセットを適切に設定すると状態推定の精度が改善する可能性があります．
今回は GNSS レシーバのオフセットのみ修正します．

![gnss](resources/setup_assistant/gnss.png)

<!--
## オプションデバイス (Camera, LiDAR, Odometry, Tether Station)

---

オプションで搭載する機器の設定を行います．以下のような機器がサポートされています:

- RGB カメラ (RGB Camera)
- 深度カメラ (Depth Camera)
- LiDAR (LiDAR)
- ドローンスパイダー (Tether Station)

今回はいずれも搭載しないためパスします．
-->

## Controller

---

制御器に関する設定を行います．
選択リストを開くと制御器のリストが表示されます．
URDF やこれまでの設定を元に使用可能な制御器のみ選択できるようになっています．
今回は`Multirotor PID`を選択します．
パラメータはデフォルトのままとします．

![controller](resources/setup_assistant/controller.png)

## Observer

---

状態推定器に関する設定を行います．
`Error State Kalman Filter`を選択し，パラメータはデフォルトのままとします．

![observer](resources/setup_assistant/observer.png)

## Hardware

---

使用するフライトコントローラハードウェアを設定します．
Tobas の最初のバージョンである Aso を選択します．

![hardware](resources/setup_assistant/hardware.png)

## PreArm Check

---

モータを起動する前のチェック項目の有効/無効の設定を行います．
基本的には全て有効にすべきですが，例えば GNSS などの位置の取得元が 1 つもない場合でも，位置に関する項目を全て無効化することで姿勢制御モードでのみ飛行させることができます．
今回は全ての項目にチェックします．

![pre_arm_check](resources/setup_assistant/pre_arm_check.png)

## Simulation

---

Gazebo のシミュレーション環境の設定を行います．
経緯度と海抜高度は，デフォルトでは日本経緯度原点と日本水準原点に設定されています．
今回はさほど重要ではないためデフォルトのままとします．
また，空力系のモデル化誤差の程度を設定でき，今回はこれもデフォルトのままとします．

![simulation](resources/setup_assistant/simulation.png)

## Author Info

---

Setup Assistant によって生成される Tobas パッケージの管理者の名前とメールアドレスを入力します．

![author_info](resources/setup_assistant/author_info.png)

## ROS Package

---

Tobas パッケージを生成するディレクトリとパッケージ名を設定します．
`Parent Directory`を colcon ワークスペースの`src/`以下に設定します．今回はデフォルトのままとします．
`Package Name`に適当なパッケージ名を設定します．こちらも今回はデフォルトのままとします．
`Generate`ボタンを押すと，指定したディレクトリに Tobas パッケージ (\*.TBS) が生成されます．

![ros_package](resources/setup_assistant/ros_package.png)
