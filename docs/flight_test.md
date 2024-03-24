# Flight Test

HIL が成功したことを確認し，実機での試験飛行を行います．

モバイルルータの電源を入れ，ラズパイに SSH 接続します．

```bash
user@pc $ ssh pi@navio
```

プロポの電源を入れ，Tobas パッケージの real.launch を起動します．

```bash
pi@navio $ su
root@navio $ roslaunch tobas_f450_config real.launch
```

HIL と同様に，プロポの E_STOP (CH5) を一度オンにしてからオフにするとドローンが操作できます．
<span style="color: red;"><strong>E_STOP をもう一度オンにすると緊急停止となり，全てのモータが停止するため注意してください．</strong></span>
プロポのピッチレバーは南北 ，ロールレバーは東西，スロットルレバーは上下の速度にそれぞれ対応しています．

<iframe width="560" height="315" src="https://www.youtube.com/embed/EldjS8AnBjw?si=mdp2SFPWEta51UOP" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
<br>
