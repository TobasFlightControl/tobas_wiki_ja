# Gazebo シミュレーション

ドローンの物理シミュレーションを行います．
Tobas パッケージの作成とハードウェアセットアップは既に済んでいることを前提とします．

## 開始と終了の手順

---

PC をフライトコントローラと同じネットワークに接続したうえで Tobas を起動し，`tobas_f450.TBS`を読み込みます．

ツールボタンから`Simulation`のページを開き，`Static Configurations`の`Simulation Type`を`HITL`に，`World`を`Standard World`の`basic`に設定します．

![simulation_settings](resources/gazebo_simulation/simulation_settings.png)

`Start`ボタンを押すと Gazebo が起動し，ドローンが原点位置に配置されます．これには数分かかることがあります．

![launch_gazebo](resources/gazebo_simulation/launch_gazebo.png)

`Terminate`ボタンを押すとシミュレーションが終了し，一定時間後に FC との通信が再開します．

## プロポでの操縦

---

1. プロポの電源を入れ，Enable スイッチをオンにします．
1. 飛行モードを`Loiter`，つまり GNSS 位置指令モードにします．
1. Pre-Arm Check の項目をすべて満たしていることを確認します．
1. スロットルレバーを下，ヨーレバーを右に傾けた状態で 5 秒間維持するとアームします．
1. アームしたら各レバーで操縦することができます．
1. スロットルレバーを下，ヨーレバーを左に傾けた状態で 2 秒間維持するとディスアームします．
