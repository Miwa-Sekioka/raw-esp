# WROOM02 raw-esp(SPI版) ホストプログラム

## ビルド方法
wroom_relayのビルド

```
% make
```

wroom_relayが作成される

実行可能にする

```
% chmod 700 wroom_relay
% chmod 700 wroom_socat.sh
```

## 常駐起動方法(systemctl)

- wroom_relay.service
 
 以下の行に、アプリのパスと、使用するSPIのパスを設定する

 .serviceファイルでは、フルパスで指定する必要あり

 Raspberry piはspidev0.0 WB15はspidev1.0

```
  WorkingDirectory=/var/wroom_relay
  ExecStart=/var/wroom_relay/wroom_relay /dev/spidev0.0
```


- サービスの登録

```
#サービスを登録する
% sudo systemctl link /var/wroom_relay/wroom_relay.service
% sudo systemctl link /var/wroom_relay/wroom_socat.service

#自動起動を有効にする
% sudo systemctl enable wroom_relay
% sudo systemctl enable wroom_socat
```


## 手動で起動する
 1. デーモン起動を解除する
 1. 個別に起動して問題が発生していないか確認する

```
./wroom_relay
./wroom_socat.sh
```


## ブラウザからアクセスする
  wroom02はssid=FACE-LOCK, pass=miwa3069 ip=192.168.4.1になります。

  WLANで接続してください。

  sshで192.168.4.1にアクセス可能です。


## ESP-WROOM02基板とCPUボードの接続方法
```
WB15            WROOM02(PIN番号とIO番号は異なります）
 PIN19(ECSPI2_MOSI) -  IO13(HSPI_MOSI) 
 PIN21(ECSPI2_MISO) -  IO12(HSPI_MISO)
 PIN23(ECSPI2_CLK)  -  IO14(HSPI_CLK)
 PIN25(GND)         -  GNG

RaspberryPI            WROOM02(PIN番号とIO番号は異なります）
 PIN19(ECSPI2_MOSI) -  IO13(HSPI_MOSI) 
 PIN21(ECSPI2_MISO) -  IO12(HSPI_MISO)
 PIN23(ECSPI2_CLK)  -  IO14(HSPI_CLK)
 PIN25(GND)         -  GNG
```
