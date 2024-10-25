# RemoconServant
## 1. はじめに
これは、BitTradeOne製 USB接続赤外線リモコンKIT専用のファームウェアです。  
メディアプレイヤーアプリケーションKodiの制御を想定しています。  
対応する赤外線リモコン送信機は以下の製品です。

- [USB接続赤外線リモコンKIT](https://bit-trade-one.co.jp/product/assemblydisk/ad00020/)
- [元ファームウェア](https://github.com/bit-trade-one/AD00020-USB_IR_Remote_Controller)
- plex製TVチューナー（w3u4等）付属リモコン

## 2. 特徴として
- 上下左右の移動ボタンは一定時間押下げでリピートします。
- 決定ボタン、音量ボタンはKodiの長押しボタンに対応しています。

## 3. キーの割り当て
ボタン名称に対応したkodiのショートカットキーを設定。  
用途のないボタンにはファンクションキーを割り当てています。  
必要な機能があればファンクションキーを利用できます。  

### Plex製リモコン
| 対応ボタン | 出力キー(ctrl:c shift:s) | 機能 |
----|----|----
|Pow			|power		|ActivateWindow(ShutdownMenu)|
|Player On		|F1 + c		|-|
|Player Off		|F2 + c		|-|
|Rec			|k			|ActivateWindow(TVRecordings)|
|Epg			|e			|ActivateWindow(TVGuide)|
|Tv				|h			|ActivateWindow(TVChannels)|
|Dvd			|b			|ActivateWindow(TVTimers)|
|Back			|backspace	|Back|
|Cc				|F3 + c		|-|
|Mute			|mute		|Mute|
|Info			|i			|info|
|音声切替		|F4 + c		|-|
|全画面			|F5 + c		|-|
|Left Arrow		|←			|Left|
|Right Arrow	|→			|Right|
|Up Arrow		|↑			|Up|
|Down Arrow		|↓			|Down|
|Ok				|enter		|Select|
|Play			|p			|Play|
|Pause			|space		|Pause|
|Stop			|x			|Stop|
|Rec			|F6 + c		|-|
|<<				|r			|Rewind|
|>>				|f			|FastForward|
|<				|,			|SkipPrevious|
|>				|.			|SkipNext|
|1				|1			|Number1|
|2				|2			|Number2|
|3				|3			|Number3|
|4				|4			|Number4|
|5				|5			|Number5|
|6				|6			|Number6|
|7				|7			|Number7|
|8				|8			|Number8|
|9				|9			|Number9|
|0				|0			|Number0|
|*				|*(6 + s)	|-|
|#				|#(3 + s)	|-|
|CH+			|pageup		|ChannelUp|
|CH-			|pagedown	|ChannelDown|
|VOl+			|volume_up	|VolumeUp|
|VOl-			|volume_down|VolumeDown|
|放送切替		|F7 + c		|-|
|クリア			|F8 + c		|-|
|入力			|F9 + c		|-|
|data			|F10 + c	|-|
|青				|F1			|-|
|赤				|F2			|-|
|緑				|F3			|-|
|黄				|F4			|-|
