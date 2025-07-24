# What is UADF

Tobas が定義している Universal Aircraft Description Format (UADF) とは，
ロボットを表現する一般的なフォーマットである Universal Robot Description Format (URDF) に，
飛翔体特有の要素を追加したフォーマットです．

## URDF とは

---

剛体リンク系で表現されるあらゆるロボットを記述するためのフォーマットです．
ロボットに含まれる各リンクの質量情報，接触情報，視覚情報に加え，それらをつなぐジョイントが定義されます．
詳しくは<a href=https://wiki.ros.org/urdf target="_blank">urdf | ROS.org</a>をご参照ください．

## UADF と URDF の違い

---

UADF では，URDF のジョイント型に加えて，飛翔体特有の以下のジョイント型が追加されています．

### thrust

モータ，プロペラを合わせた推進モジュールを表すジョイントです．
URDF の`continuous`がベースですが，以下の点が異なります．

- `axis`: 回転軸．軸方向に推力が出る想定．
- `direction`: 回転方向．`value`に`cw`または`ccw`を指定する．

※ `thrust`ジョイントはエンドジョイントでなければなりません．

### cs

固定翼機の操舵面 (Control Surface) を表すジョイントです．
URDF の`revolute`がベースになっています．

※ `cs`ジョイントはエンドジョイントでなければなりません．

### tilt

アクティブチルトロータのチルトジョイントを表します．
URDF の`revolute`がベースになっています．

※ `tilt`ジョイントの先には 1 つの`thrust`ジョイントが接続していなければなりません．

## UADF の作成例

---

`/opt/tobas/share/tobas_description/urdf/`以下のファイル (\*.uadf) をご参照ください．
