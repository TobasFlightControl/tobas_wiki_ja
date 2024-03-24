# Tobas ユーザガイド

Tobas は，高度な機能を持つドローンの開発を促進するために作られた，ドローン開発支援ツールです．
<a href=http://forestofazumino.web.fc2.com/ros/ros_urdf_xacro.html target="_blank">URDF (Unified Robot Description Format)</a>
を元に詳細なモデル化を行うため，制御性能を向上させ，従来のフライトコントローラでは飛ばせないようなドローンでも飛ばすことができます．

## 特徴

---

- URDF を元に，機体の構造や質量特性を考慮
- 推進系や固定翼の空力特性を考慮
- モータと推進系の連成による非線形ダイナミクスを考慮
- 関節角の変化によるダイナミクスの変化を考慮
- 各種設定を行うための GUI
- ROS に対応し，シミュレーションと実機で同じインターフェースを提供

## 何ができるか

---

### 制御性能の向上・機体設計の幅を拡大

機体の重心やプロペラの配置等を考慮する Tobas は制御器の自由度が高く，従来のフライトコントローラよりも優れた制御性能を提供できる可能性があります．

また，URDF で表現でき，対応した制御器が存在すればどのような機体でも飛ばすことができます．
例えば，Tobas は以下のような変則的な機体にも対応しています:

- 特殊なセンサにより重心が中心部から大きく外れた機体
- カメラの画角を確保するためにプロペラの配置が点対称でない機体
- プロペラが同一平面上に並ばない機体
- ロボットアームを搭載した機体

### ゲイン調整の手間を削減

機体を正しくモデル化することで，並進系と回転系のダイナミクスを機体に非依存の形で取り出せ，事前に解析することができます．
それにより Tobas では予め無難なゲインが設定されており，ユーザは基本的にゲイン調整をすることなく機体を飛ばすことができます．

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

## 前提知識

---

### Linux

Tobas は Linux ディストリビューションの 1 つである Ubuntu 20.04 LTS で動作します．
コマンドラインについて最低限の知識をもっていることを想定しています．

### ROS

Tobas は内部通信や API に ROS (Robot Operating System)</a>を使用します．
ROS の知識がなくてもドローンを飛ばすことはできますが，Tobas の機能を最大限利用するために ROS の知識を持っておくことを勧めます．
以下が参考になります:

- <a href=https://wiki.ros.org/ja target="_blank">ROS Wiki</a>
- <a href=https://www.kohgakusha.co.jp/books/detail/978-4-7775-2168-5 target="_blank">ROS ではじめるロボットプログラミング</a>
- <a href=https://www.oreilly.co.jp/books/9784873118093 target="_blank">プログラミング ROS</a>

### 3D CAD

Tobas を使用するためには，ドローンの 3D モデルを元に URDF を作成する必要があります．
3D モデルがなくても URDF を作成することはできますが，パーツの位置関係や質量特性の計算の際に CAD が使えると便利です．
例えば以下のようなものがあります:

- <a href=https://www.autodesk.co.jp/products/fusion-360 target="_blank">Fusion 360</a>
- <a href=https://www.autodesk.co.jp/products/inventor target="_blank">Inventor</a>
- <a href=https://www.solidworks.com/ja target="_blank">SolidWorks</a>

## 連絡先

---

Tobas のご利用をお考えの方は，下記までご連絡いただけますと幸いです．

土肥 正義<br>
E-mail: masa0u0masa1215(at)gmail.com<br>
Tel: 070-8484-1557<br>
