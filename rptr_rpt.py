# this module will be imported in the into your flowgraph

import math
import os

###########################################################################
#                       Global variable -> delay():                       #
###########################################################################

t = 0
f = 0
d = 0
gpo_status = 0
tx = '3'
###########################################################################
#                      Global variable -> newrssi():                      #
###########################################################################

g = 73


###########################################################################
#                             Funtion hang TX                             #
###########################################################################
def hang(probe, delay):
	global t, f, d, gpo_status, tx
	if probe:
		probe = probe
	else:
		probe = 0

	if probe != 0:
		t = 1
		f = 1
		d = 0
		if gpo_status == 0:
			os.system('iio_attr -u ip:pluto.local -D ad9361-phy gpo_set  3-1')
			gpo_status = 1
	elif f == 1 and probe == 0:
		if d < (delay):
			d = d + 100
			t = 1
			f = 1
		else:
			t = 0
			f = 0
			d = 0
			if gpo_status == 1:
				os.system('iio_attr -u ip:pluto.local -D ad9361-phy gpo_set 3-0')
				gpo_status = 0
	return t

############################################################################
#			     Function AGC regolate gain Pluto                          #
############################################################################
def agc(rssi):
	global g
	sa = math.trunc(rssi)
	if sa > -73:
		g = g - 6
	elif sa < -88:
		if g <= 67:
			g = g + 6
		else:
			g = 73
	else:
		g = g
	return g

###########################################################################
#			     Function new RSSI with addition agc                      #
###########################################################################
def newrssi(s):
	global g
	ss = math.trunc(s)
	ns = ss + (73 - g)
	return ns

###########################################################################
#                         trigger status input                            #
###########################################################################
def status(input):
	if input != 0:
		return 1
	else:
		return 0

###########################################################################
#                         CTCSS enable disable                            #
###########################################################################
def ctcss_en(input):
	if input > 0.0:
		return 0
	else:
		return 1

###########################################################################
#                                 END                                     #
###########################################################################
