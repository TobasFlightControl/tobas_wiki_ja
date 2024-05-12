# Flight Test

シミュレーションが成功したことを確認し，実機での試験飛行を行います．

<iframe width="560" height="315" src="https://www.youtube.com/embed/EldjS8AnBjw?si=mdp2SFPWEta51UOP" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<br>

## 開始の手順

---

1. フライトコントローラと ESC に適切に給電します．
1. シミュレーションと同様に，プロポの E_STOP (CH5) を一度オンにしてからオフにするとドローンが操作できます．
1. `Mission Planner` `Control System` `Parameter Tuning`等の機能も，シミュレーションと同様に使用することができます．

<span style="color: red;"><strong>警告: アーム後に E_STOP をオンにすると，全てのモータが緊急停止します．</strong></span>

## 終了の手順

---

1. ドローンを安全に着陸させます．
1. E_STOP (CH5) をオンにし，モータを停止します．
1. 右上の`Shutdown`ボタンからラズパイと GUI をシャットダウンします．
1. ラズパイとモータへの給電を切ります．
