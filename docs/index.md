# Tobas ユーザガイド

Tobas は，近年のドローン市場の拡大に伴う機体の大型化と特殊化に対応するために開発された，モデルベースの新しいフライトコントローラです．
従来のフライトコントローラと異なり，個々の機体の物理特性を詳細に考慮して制御系を設計するため，
従来のフライトコントローラでは難しい機体でも精度良く飛ばすことができます．

## 特徴

---

- <a href=http://forestofazumino.web.fc2.com/ros/ros_urdf_xacro.html target="_blank">URDF</a> を元に，機体の構造や質量特性を考慮
- 推進系や固定翼の空力特性を考慮
- モータと推進系の連成による非線形ダイナミクスを考慮
- 関節角の変化によるダイナミクスの変化を考慮
- 突風やモデル化誤差などの外乱補償
- 完全 GUI でセットアップ可能
- ROS に対応し，シミュレーションと実機で同じインターフェースを提供

## 何ができるか

---

### 制御性能の向上・機体設計の幅を拡大

機体の重心やプロペラの配置等を考慮する Tobas は制御器の自由度が高く，従来のフライトコントローラよりも優れた制御性能を提供できる可能性があります．

また，URDF で表現でき，対応した制御器が存在すればどのような機体でも飛ばすことができます．
例えば，Tobas は以下のような変則的な機体にも対応しています:

- 特殊なセンサにより重心が中心部から大きく外れた機体
- カメラの画角を確保するためにプロペラの配置が非対称な機体
- ティルトロータを搭載した機体
- ロボットアームを搭載した機体

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

## 使用例

---

### クアッドコプター

典型的なクアッドコプターです．
DJI F450 のフレームキットを使用しています．

<iframe width="560" height="315" src="https://www.youtube.com/embed/EldjS8AnBjw?si=mdp2SFPWEta51UOP" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
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
