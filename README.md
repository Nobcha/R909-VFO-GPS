# R909-VFO-GPS

I would like to study GPS module. So I will divert R909-VFO-1602 PCB for the trial base for this GPS corrected VFO.

At first I connected the GPS via SoftwareSerial and show the longtituse and latitude on the 1602 LCD. 
"https://github.com/Nobcha/R909-VFO-GPS/blob/main/GPS_TEST20241022_3.ino"

Secondaly I made GPS powered clock. "https://github.com/Nobcha/R909-VFO-GPS/blob/main/GPS_watch_kpa.ino"

And I made GPS corrected VFO. "https://github.com/Nobcha/R909-VFO-GPS/blob/main/Si5351_GPS-kpa.ino"

I adopted E108 GN02D GPS module and connected it to R909-VFO via SoftwareSerial.

I will introduce the sketch of the trial connection with GPS and R909-VFO refered 
"https://github.com/W3PM/GPS-Si5351-VFO-QEX-JUL-AUG-2015"

However there are some issues of this one. ie, rotary encoder responding, getting the correction frequency, and so.
I renewed the sketch which is the improved SQ1GU's version from "http://sq1gu.tobis.com.pl/pl/syntezery-dds"
The latest sketch is here. "https://github.com/Nobcha/R909-VFO-GPS/blob/main/Si5351_GPS_kpa.ino"

I designed the PCB for this project. Please find uplorded Gerber files.

I wrote the operation and assembling manual as below.
https://github.com/Nobcha/R909-VFO-GPS/blob/main/R909-VFO%3DGPS1602_manual_EN.pdf

https://github.com/user-attachments/assets/a366d376-8fa3-4aea-8705-ed7749ec5e32


Ｒ９０９と言うのはエアバンド受信機などのＤＩＹシリーズです。Ｒ９０９－ＤＳＰはＡｒｄｕｉｎｏ制御でＳｉ４７３２，Ｓｉ５３５１ａモジュールを使ったエアバンド受信機なのですが、このＶＦＯ部を取り出したものがＲ９０９－ＶＦＯです。Ｒ９０９－ＶＦＯにはＯＬＥＤ表示版と１６０２ＡＬＣＤ表示版があります。今回はこの１６０２ＡＬＣＤ版のＲ９０９－ＶＦＯとＧＰＳモジュールを組み合わせた試作を行いました。”https://github.com/Nobcha/R909-SDR/blob/main/R909-SDR-1602_BD.jpg”
Ｅ１０８　ＧＮ０２Ｄと言うちょっと旧型のＧＰＳモジュールをつなぎ、緯度経度表示とか、時計機能とか、ＧＰＳデータで周波数較正するR909-VFO-GPSを試作しましたので、ご紹介します。
この基板を使います。”https://github.com/Nobcha/R909-SDR/blob/main/5531_debugPanel(3).zip”　”https://github.com/Nobcha/R909-SDR/blob/main/5531_debug_panel_scm.pdf”　”https://github.com/Nobcha/R909-SDR/blob/main/5531_debug_panel_bom.pdf”

GPS接続に対応した基板を作成しました。基板製作データをアップロードしました。スケッチや使い方に関するマニュアルを作りました。
https://github.com/Nobcha/R909-VFO-GPS/blob/main/R909-VFO%3DGPS1602_manual_JA.pdf
