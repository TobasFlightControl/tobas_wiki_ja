# Tobas ユーザガイド

Tobas は，近年のドローン市場の拡大に伴う機体の大型化と特殊化に対応するために開発された，モデルベースの新しいフライトコントローラです．
従来のフライトコントローラと異なり，個々の機体の物理特性を詳細に考慮して制御系を設計するため，
従来のフライトコントローラでは難しい機体でも精度良く飛ばすことができます．

## 特徴

---

- <a href=https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html target="_blank">URDF</a> を元に，機体の構造や質量特性を考慮
- 推進系や固定翼の空力特性を考慮
- モータと推進系の連成による非線形ダイナミクスを考慮
- 関節角の変化による自重・反力補償
- 突風や地面効果などの外乱補償
- 完全 GUI でセットアップ可能
- ROS に対応し，シミュレーションと実機で同じインターフェースを提供

## 何ができるか

---

### 制御性能の向上

Tobas は GUI に入力したユーザの機体構造から完全な 6 自由度運動方程式を構築して制御を行うため，　
従来のフライトコントローラよりも優れた制御性能を発揮できます．
例えば以下のような情報を GUI に入力します．

- 機体を構成する各剛体リンクの力学パラメータ: 質量，重心，慣性テンソル
- バッテリーのスペック: セル数，放電容量，放電率
- モータのスペック: KV 値，内部抵抗，磁極数
- プロペラのスペック: 直径，ピッチ，UIUC のデータ

### 機体設計の幅を拡大

機体構造が URDF で表現でき，対応した制御器が存在すればどのような機体でも飛ばすことができます．
例えば，Tobas は以下のような変則的な機体にも対応しています．

- 特殊なセンサにより重心が中心部から大きく外れた機体
- カメラの画角を確保するためにプロペラの配置が非対称な機体
- ロボットアームを搭載した機体
- ティルトロータを搭載した機体

### ゲイン調整の手間を削減

機体を正しくモデル化することで，並進系と回転系のダイナミクスを機体に非依存の形で取り出せ，事前に解析することができます．
それにより Tobas では予め無難なゲインが設定されており，ユーザは基本的にゲイン調整をすることなく機体を飛ばすことができます．
また，必要であれば全てのパラメータを飛行中にオンラインで調整することができます．

### 現実に即したシミュレーション

機体の質量特性や推進系の空力特性を考慮しているため，現実に即した物理シミュレーションが可能です．
それにより，実機試験のコストを大幅に削減することができます．

上記以外にも飛行性能に影響を与える要素をシミュレーションできます．例えば以下のようなものがあります:

- 風 (定常風，乱流，突風)
- 機体のモデル化誤差
- バッテリーの電圧降下
- ESC の最大電流
- センサの遅延，ノイズ

## Flight Management Unit (FMU)

### Tobas Aso

<img src="../resources/introduction/aso_1.png" alt="aso_1" width="49%"> <img src="../resources/introduction/aso_2.png" alt="aso_2" width="49%">

#### IC

- 6-axis IMU: <a href=https://www.st.com/ja/mems-and-sensors/ism330dlc.html target="_blank">ISM330DLC | STMicroelectronics</a>
- Magnetometer: <a href=https://www.st.com/ja/mems-and-sensors/iis2mdc.html target="_blank">IIS2MDC | STMicroelectronics</a>
- Barometer: <a href=https://www.st.com/ja/mems-and-sensors/ilps22qs.html target="_blank">ILPS22QS | STMicroelectronics</a>
- GNSS Receiver: <a href=https://www.u-blox.com/en/product/zed-f9p-module target="_blank">ZED-F9P-15B | u-blox</a>
- A/D Converter: <a href=https://www.ti.com/product/ja-jp/INA228 target="_blank">INA228 | Texas Instruments</a>
- Motor Driver: <a href=https://www.st.com/ja/microcontrollers-microprocessors/stm32h7a3-7b3.html target="_blank">STM32H7A3 | STMicroelectronics</a>

#### Interface

- GNSS Antenna: SMA
- Power Module: Molex 6pin 2.0mm
- UART, I2C Interface: JST-GH 6pin

## 使用例

---

### クアッドコプター

典型的なクアッドコプターです．
DJI F450 のフレームキットを使用しています．

<iframe width="560" height="315" src="https://www.youtube.com/embed/sHoA8yKJPs4?si=CCOEPsu6z9hd7zOb" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe> 
<br>

### 非平面ロータ配置ヘキサコプター

全てのプロペラが水平面から 30 度傾いているヘキサコプターです．
平面ロータ配置のマルチコプターは位置を変化させるために姿勢を変化させる必要があるのに対し，
非平面ロータ配置マルチコプターは位置と姿勢を独立に制御することができます．
そのため，地面と平行の姿勢を保ったまま平行移動したり，ホバリングしたまま姿勢を変化させることができます．
また，直接水平方向に推力を発生させられるため位置決め精度が高く，耐風性能にも優れています．

<iframe width="560" height="315" src="https://www.youtube.com/embed/1RIXLGmx1RA?si=ADkOlZsAMb1tHyNr" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<br>

### ロボットアームドローン

4 軸のロボットアームを搭載した，非平面ロータ配置ヘキサコプターです．
アームにより発生する反力や重心の変化を動的に保証することで，
アームを大きく振り回しても位置姿勢を一定範囲内に留めることができています．

<iframe width="474" height="843" src="https://www.youtube.com/embed/L7nRlG1OgyY" title="Tobas | The Drone That Stays Stable with a Swinging Arm (2024/02/15) #drone" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<br>

### アクティブティルトヘキサコプター

全てのアームを両方向に 120 度ずつ回転させられるヘキサコプターです．
ホバリングした状態で姿勢を大きく変化させられるのが特徴です．
取り付けた検査機器等を任意の姿勢で保持することができるため，例えば傾いた壁面に対する非破壊検査への応用が期待されます．

<iframe width="560" height="315" src="https://www.youtube.com/embed/UYwoFjf6ubc?si=RsDKgr98DVvdhaWB" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
<br>

## PC のシステム要件 <!-- cf. https://www.solidworks.com/ja/support/system-requirements -->

---

| 要件 | 必須                           | 推奨               |
| :--- | :----------------------------- | ------------------ |
| OS   | Ubuntu 24.04 LTS (ROS 2 Jazzy) |                    |
| RAM  | 8GB                            | 16GB               |
| CPU  | AMD64 (x86-64)                 |                    |
| GPU  |                                | NVIDIA GeForce RTX |

<br>

## 連絡先

---

Tobas のご利用をお考えの方は，下記までご連絡いただけますと幸いです．

土肥 正義<br>
E-mail: masa0u0masa1215(at)gmail.com<br>
Tel: 070-8484-1557<br>
