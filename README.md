# NBFM_Repeater
Simple NBFM RepeaterSDR with ADALM PLUTO based on GNURadio >= 3.10 python 3.

Addictions:
- Python 3 >
- gr-iio
- gr-audio
- gr-qtgui
- gr-analog
- libiio >(0.23)

Feature system developement:
- AGC control
- CTCSS decode
- Audio Beacon ID
- Used GPO3 Pluto for driver RF power amplifier frontend
- Linear BF 50-3000hz

It's possible change Paramenter on .ini file or small GUI interface.
I hope this project can be an inspiration for someone and can grow and become something more.

![alt text](https://github.com/lucamarche-iz1mlt/NBFM_Repeater/blob/main/Image/NBFM_Repeater%20GNUR%20diagram.jpg)

Reference ADALM-PLUTO:

https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/adalm-pluto.html#eb-overview

https://wiki.analog.com/university/tools/pluto/drivers/linux

https://github.com/orgs/analogdevicesinc/repositories

Come and use GPO PLUTO:

-	Set Gpo in manual mode
		
		iio_attr -u ip:pluto.local -D ad9361-phy adi,gpo-manual-mode-enable 1
		
-	Set GPO pin ouput

		iio_attr -u ip:pluto.local -D ad9361-phy gpo_set <gpo 0:3>-<state 0:1>
		
More Reference:

https://wiki.analog.com/resources/tools-software/linux-drivers/iio-transceiver/ad9361-customization


Beacon id config

- Create a VAC in the following way

		pacmd load-module module-null-sink sink_name=Virtual_Sink sink_properties=device.description=Virtual_Sink
		
Now set VAC as default audio and so you can launch through any audio player of your beacon and through "crontab" to play the cyclically
