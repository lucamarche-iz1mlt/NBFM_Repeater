#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: NBFM_Repeater
# GNU Radio version: 3.10.1.1

from packaging.version import Version as StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print("Warning: failed to XInitThreads()")

from PyQt5 import Qt
from PyQt5.QtCore import QObject, pyqtSlot
from gnuradio import eng_notation
from gnuradio import analog
from gnuradio import audio
from gnuradio import blocks
import math
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import iio
from gnuradio import qtgui
from gnuradio.qtgui import Range, RangeWidget
from PyQt5 import QtCore
import configparser
import rptr_rpt as rpt  # embedded python module
import time
import threading



from gnuradio import qtgui

class rptr(gr.top_block, Qt.QWidget):

    def __init__(self, uri='ip:pluto.local'):
        gr.top_block.__init__(self, "NBFM_Repeater", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("NBFM_Repeater")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "rptr")

        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except:
            pass

        ##################################################
        # Parameters
        ##################################################
        self.uri = uri

        ##################################################
        # Variables
        ##################################################
        self.func_rssi = func_rssi = 0
        self.func_nbfm_busy = func_nbfm_busy = 0
        self._conf_hangtime_config = configparser.ConfigParser()
        self._conf_hangtime_config.read('./rptr.ini')
        try: conf_hangtime = self._conf_hangtime_config.getint('main', 'hangtime')
        except: conf_hangtime = 1300
        self.conf_hangtime = conf_hangtime
        self._conf_ctcss_config = configparser.ConfigParser()
        self._conf_ctcss_config.read('./rptr.ini')
        try: conf_ctcss = self._conf_ctcss_config.getfloat('main', 'ctcss')
        except: conf_ctcss = 0.0
        self.conf_ctcss = conf_ctcss
        self.samp_bf = samp_bf = 16000
        self.rx_status = rx_status = rpt.status(func_nbfm_busy)
        self.rs = rs = rpt.newrssi(func_rssi)
        self.pl_freq = pl_freq = conf_ctcss
        self.func_prb_beacon_busy = func_prb_beacon_busy = 0
        self.func_ctcssdec = func_ctcssdec = 0
        self.func_bfbusy = func_bfbusy = 0
        self.delay = delay = conf_hangtime
        self._conf_trxcal_config = configparser.ConfigParser()
        self._conf_trxcal_config.read('./rptr.ini')
        try: conf_trxcal = self._conf_trxcal_config.getint('main', 'trxcal')
        except: conf_trxcal = 1400
        self.conf_trxcal = conf_trxcal
        self._conf_squelch_config = configparser.ConfigParser()
        self._conf_squelch_config.read('./rptr.ini')
        try: conf_squelch = self._conf_squelch_config.getfloat('main', 'squelch')
        except: conf_squelch = -40
        self.conf_squelch = conf_squelch
        self._conf_offset_config = configparser.ConfigParser()
        self._conf_offset_config.read('./rptr.ini')
        try: conf_offset = self._conf_offset_config.getfloat('main', 'offset')
        except: conf_offset = -60000
        self.conf_offset = conf_offset
        self._conf_freq_config = configparser.ConfigParser()
        self._conf_freq_config.read('./rptr.ini')
        try: conf_freq = self._conf_freq_config.getint('main', 'freq')
        except: conf_freq = 145000000
        self.conf_freq = conf_freq
        self.sq_lvl = sq_lvl = conf_squelch
        self.shift = shift = 15000
        self.samp_rate = samp_rate = samp_bf*9
        self.ptt = ptt = rpt.hang(func_bfbusy, delay)
        self.pluto_gain = pluto_gain = rpt.agc(func_rssi)
        self.oscref = oscref = conf_trxcal
        self.offset = offset = conf_offset
        self.max_dev = max_dev = 5000
        self.freq = freq = conf_freq
        self.filter_wide = filter_wide = 7500
        self.ctcss_status = ctcss_status = rpt.status(func_ctcssdec)
        self.ctcss_en = ctcss_en = rpt.ctcss_en(pl_freq)
        self.buffer = buffer = 8192
        self.beacon_status = beacon_status = rpt.status(func_prb_beacon_busy)
        self.beacon_att = beacon_att = rpt.beacon_att(rx_status)
        self.RSSI = RSSI = rs

        ##################################################
        # Blocks
        ##################################################
        self._sq_lvl_range = Range(-150, 0, 1, conf_squelch, 100)
        self._sq_lvl_win = RangeWidget(self._sq_lvl_range, self.set_sq_lvl, "Squelch", "counter", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._sq_lvl_win)
        self.rssi = blocks.probe_signal_f()
        self.prb_nbfm_busy = blocks.probe_signal_f()
        self.prb_ctcssdec = blocks.probe_signal_f()
        self.prb_bfbusy = blocks.probe_signal_f()
        self.prb_beacon_busy = blocks.probe_signal_f()
        # Create the options list
        self._pl_freq_options = [0.0, 67.0, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8, 97.4, 100.0, 103.5, 107.2, 110.9, 114.8, 118.8, 123.0, 127.3, 131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 162.2, 167.9, 173.8, 179.9, 186.2, 192.8, 203.5, 210.7, 218.1, 225.7, 233.6, 241.8, 250.3]
        # Create the labels list
        self._pl_freq_labels = ['0.0', '67.0', '71.9', '74.4', '77.0', '79.7', '82.5', '85.4', '88.5', '91.5', '94.8', '97.4', '100.0', '103.5', '107.2', '110.9', '114.8', '118.8', '123.0', '127.3', '131.8', '136.5', '141.3', '146.2', '151.4', '156.7', '162.2', '167.9', '173.8', '179.9', '186.2', '192.8', '203.5', '210.7', '218.1', '225.7', '233.6', '241.8', '250.3']
        # Create the combo box
        self._pl_freq_tool_bar = Qt.QToolBar(self)
        self._pl_freq_tool_bar.addWidget(Qt.QLabel("PL Tone" + ": "))
        self._pl_freq_combo_box = Qt.QComboBox()
        self._pl_freq_tool_bar.addWidget(self._pl_freq_combo_box)
        for _label in self._pl_freq_labels: self._pl_freq_combo_box.addItem(_label)
        self._pl_freq_callback = lambda i: Qt.QMetaObject.invokeMethod(self._pl_freq_combo_box, "setCurrentIndex", Qt.Q_ARG("int", self._pl_freq_options.index(i)))
        self._pl_freq_callback(self.pl_freq)
        self._pl_freq_combo_box.currentIndexChanged.connect(
            lambda i: self.set_pl_freq(self._pl_freq_options[i]))
        # Create the radio buttons
        self.top_layout.addWidget(self._pl_freq_tool_bar)
        self._oscref_range = Range(-50000, 50000, 50, conf_trxcal, 10)
        self._oscref_win = RangeWidget(self._oscref_range, self.set_oscref, "OSC REF", "counter", int, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._oscref_win)
        self._offset_range = Range(-10000000, 10000000, 25000, conf_offset, 100)
        self._offset_win = RangeWidget(self._offset_range, self.set_offset, "Shift RX", "counter", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._offset_win)
        self._freq_range = Range(144000000, 1300000000, 12500, conf_freq, 100)
        self._freq_win = RangeWidget(self._freq_range, self.set_freq, "Freq TX", "counter", int, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._freq_win)
        self._filter_wide_range = Range(4000, 12000, 1000, 7500, 50)
        self._filter_wide_win = RangeWidget(self._filter_wide_range, self.set_filter_wide, "Filter", "counter_slider", int, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._filter_wide_win)
        self.qtgui_ledindicator_0_0_0_0_0 = self._qtgui_ledindicator_0_0_0_0_0_win = qtgui.GrLEDIndicator("BEACON", "lime", "gray", beacon_status, 40, 1, 1, 2, self)
        self.qtgui_ledindicator_0_0_0_0_0 = self._qtgui_ledindicator_0_0_0_0_0_win
        self.top_layout.addWidget(self._qtgui_ledindicator_0_0_0_0_0_win)
        self.qtgui_ledindicator_0_0_0_0 = self._qtgui_ledindicator_0_0_0_0_win = qtgui.GrLEDIndicator("CTCSS", "yellow", "gray", ctcss_status, 40, 1, 1, 3, self)
        self.qtgui_ledindicator_0_0_0_0 = self._qtgui_ledindicator_0_0_0_0_win
        self.top_layout.addWidget(self._qtgui_ledindicator_0_0_0_0_win)
        self.qtgui_ledindicator_0_0_0 = self._qtgui_ledindicator_0_0_0_win = qtgui.GrLEDIndicator("RX", "green", "gray", rx_status, 40, 1, 1, 1, self)
        self.qtgui_ledindicator_0_0_0 = self._qtgui_ledindicator_0_0_0_win
        self.top_layout.addWidget(self._qtgui_ledindicator_0_0_0_win)
        self.qtgui_ledindicator_0_0 = self._qtgui_ledindicator_0_0_win = qtgui.GrLEDIndicator("TX", "red", "gray", ptt, 40, 1, 1, 3, self)
        self.qtgui_ledindicator_0_0 = self._qtgui_ledindicator_0_0_win
        self.top_layout.addWidget(self._qtgui_ledindicator_0_0_win)
        self.iio_pluto_source_0 = iio.fmcomms2_source_fc32(uri if uri else iio.get_pluto_uri(), [True, True], buffer)
        self.iio_pluto_source_0.set_len_tag_key('packet_len')
        self.iio_pluto_source_0.set_frequency((int)(freq)+(shift)+(oscref)+(offset))
        self.iio_pluto_source_0.set_samplerate(samp_rate)
        self.iio_pluto_source_0.set_gain_mode(0, 'manual')
        self.iio_pluto_source_0.set_gain(0, pluto_gain)
        self.iio_pluto_source_0.set_quadrature(True)
        self.iio_pluto_source_0.set_rfdc(True)
        self.iio_pluto_source_0.set_bbdc(True)
        self.iio_pluto_source_0.set_filter_params('Auto', '', (float)(samp_rate)/(4), (float)(samp_rate)/(3))
        self.iio_pluto_sink_0 = iio.fmcomms2_sink_fc32(uri if uri else iio.get_pluto_uri(), [True, True], buffer, False)
        self.iio_pluto_sink_0.set_len_tag_key('')
        self.iio_pluto_sink_0.set_bandwidth(200000)
        self.iio_pluto_sink_0.set_frequency((int)(freq)+(oscref))
        self.iio_pluto_sink_0.set_samplerate(samp_rate)
        self.iio_pluto_sink_0.set_attenuation(0, 0)
        self.iio_pluto_sink_0.set_filter_params('Auto', '', 10000, 40000)
        def _func_rssi_probe():
          while True:

            val = self.rssi.level()
            try:
              try:
                self.doc.add_next_tick_callback(functools.partial(self.set_func_rssi,val))
              except AttributeError:
                self.set_func_rssi(val)
            except AttributeError:
              pass
            time.sleep(1.0 / (6))
        _func_rssi_thread = threading.Thread(target=_func_rssi_probe)
        _func_rssi_thread.daemon = True
        _func_rssi_thread.start()
        def _func_prb_beacon_busy_probe():
          while True:

            val = self.prb_beacon_busy.level()
            try:
              try:
                self.doc.add_next_tick_callback(functools.partial(self.set_func_prb_beacon_busy,val))
              except AttributeError:
                self.set_func_prb_beacon_busy(val)
            except AttributeError:
              pass
            time.sleep(1.0 / (10))
        _func_prb_beacon_busy_thread = threading.Thread(target=_func_prb_beacon_busy_probe)
        _func_prb_beacon_busy_thread.daemon = True
        _func_prb_beacon_busy_thread.start()
        def _func_nbfm_busy_probe():
          while True:

            val = self.prb_nbfm_busy.level()
            try:
              try:
                self.doc.add_next_tick_callback(functools.partial(self.set_func_nbfm_busy,val))
              except AttributeError:
                self.set_func_nbfm_busy(val)
            except AttributeError:
              pass
            time.sleep(1.0 / (10))
        _func_nbfm_busy_thread = threading.Thread(target=_func_nbfm_busy_probe)
        _func_nbfm_busy_thread.daemon = True
        _func_nbfm_busy_thread.start()
        def _func_ctcssdec_probe():
          while True:

            val = self.prb_ctcssdec.level()
            try:
              try:
                self.doc.add_next_tick_callback(functools.partial(self.set_func_ctcssdec,val))
              except AttributeError:
                self.set_func_ctcssdec(val)
            except AttributeError:
              pass
            time.sleep(1.0 / (10))
        _func_ctcssdec_thread = threading.Thread(target=_func_ctcssdec_probe)
        _func_ctcssdec_thread.daemon = True
        _func_ctcssdec_thread.start()
        def _func_bfbusy_probe():
          while True:

            val = self.prb_bfbusy.level()
            try:
              try:
                self.doc.add_next_tick_callback(functools.partial(self.set_func_bfbusy,val))
              except AttributeError:
                self.set_func_bfbusy(val)
            except AttributeError:
              pass
            time.sleep(1.0 / (10))
        _func_bfbusy_thread = threading.Thread(target=_func_bfbusy_probe)
        _func_bfbusy_thread.daemon = True
        _func_bfbusy_thread.start()
        self.filter_fft_low_pass_filter_0 = filter.fft_filter_fff(1, firdes.low_pass(1, samp_rate, 50, 25, window.WIN_HAMMING, 6.76), 1)
        self._delay_range = Range(100, 5000, 100, conf_hangtime, 10)
        self._delay_win = RangeWidget(self._delay_range, self.set_delay, "TX Coda ms", "counter", int, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._delay_win)
        self.blocks_selector_0_0 = blocks.selector(gr.sizeof_float*1,ctcss_en,0)
        self.blocks_selector_0_0.set_enabled(True)
        self.blocks_selector_0 = blocks.selector(gr.sizeof_float*1,0,ctcss_en)
        self.blocks_selector_0.set_enabled(True)
        self.blocks_nlog10_ff_0 = blocks.nlog10_ff(10, 1, -59)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_ff(beacon_att)
        self.blocks_freqshift_cc_0 = blocks.rotator_cc(2.0*math.pi*shift/samp_rate)
        self.blocks_complex_to_mag_squared_0 = blocks.complex_to_mag_squared(1)
        self.blocks_add_xx_0 = blocks.add_vff(1)
        self.band_pass_filter_0_0_0 = filter.fir_filter_ccc(
            1,
            firdes.complex_band_pass(
                1,
                samp_rate,
                -filter_wide,
                filter_wide,
                2000,
                window.WIN_HAMMING,
                6.76))
        self.band_pass_filter_0_0 = filter.fir_filter_ccc(
            1,
            firdes.complex_band_pass(
                1,
                samp_rate,
                -filter_wide,
                filter_wide,
                2000,
                window.WIN_HAMMING,
                6.76))
        self.band_pass_filter_0 = filter.interp_fir_filter_fff(
            1,
            firdes.band_pass(
                1,
                samp_bf,
                50,
                3500,
                100,
                window.WIN_HAMMING,
                6.76))
        self.audio_source_0 = audio.source(samp_bf, '', True)
        self.analog_simple_squelch_cc_0 = analog.simple_squelch_cc(sq_lvl+59, 0.005)
        self.analog_pwr_squelch_xx_0 = analog.pwr_squelch_cc((int)(-ptt)*(100), 1e-4, 0, False)
        self.analog_nbfm_tx_0 = analog.nbfm_tx(
        	audio_rate=samp_bf,
        	quad_rate=samp_rate,
        	tau=7.5e-6,
        	max_dev=max_dev,
        	fh=0,
                )
        self.analog_nbfm_rx_0 = analog.nbfm_rx(
        	audio_rate=samp_bf,
        	quad_rate=samp_rate,
        	tau=7.5e-6,
        	max_dev=max_dev,
          )
        self.analog_ctcss_squelch_ff_0 = analog.ctcss_squelch_ff(samp_bf, pl_freq, 0.015, 0, 0, False)
        self._RSSI_tool_bar = Qt.QToolBar(self)

        if None:
            self._RSSI_formatter = None
        else:
            self._RSSI_formatter = lambda x: repr(x)

        self._RSSI_tool_bar.addWidget(Qt.QLabel("RSSI dBm:"))
        self._RSSI_label = Qt.QLabel(str(self._RSSI_formatter(self.RSSI)))
        self._RSSI_tool_bar.addWidget(self._RSSI_label)
        self.top_layout.addWidget(self._RSSI_tool_bar)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_ctcss_squelch_ff_0, 0), (self.blocks_selector_0_0, 0))
        self.connect((self.analog_ctcss_squelch_ff_0, 0), (self.prb_ctcssdec, 0))
        self.connect((self.analog_nbfm_rx_0, 0), (self.blocks_selector_0, 0))
        self.connect((self.analog_nbfm_rx_0, 0), (self.prb_nbfm_busy, 0))
        self.connect((self.analog_nbfm_tx_0, 0), (self.analog_pwr_squelch_xx_0, 0))
        self.connect((self.analog_pwr_squelch_xx_0, 0), (self.band_pass_filter_0_0_0, 0))
        self.connect((self.analog_simple_squelch_cc_0, 0), (self.analog_nbfm_rx_0, 0))
        self.connect((self.audio_source_0, 0), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.audio_source_0, 0), (self.prb_beacon_busy, 0))
        self.connect((self.band_pass_filter_0, 0), (self.analog_nbfm_tx_0, 0))
        self.connect((self.band_pass_filter_0_0, 0), (self.analog_simple_squelch_cc_0, 0))
        self.connect((self.band_pass_filter_0_0, 0), (self.blocks_complex_to_mag_squared_0, 0))
        self.connect((self.band_pass_filter_0_0_0, 0), (self.iio_pluto_sink_0, 0))
        self.connect((self.blocks_add_xx_0, 0), (self.band_pass_filter_0, 0))
        self.connect((self.blocks_add_xx_0, 0), (self.prb_bfbusy, 0))
        self.connect((self.blocks_complex_to_mag_squared_0, 0), (self.filter_fft_low_pass_filter_0, 0))
        self.connect((self.blocks_freqshift_cc_0, 0), (self.band_pass_filter_0_0, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.blocks_add_xx_0, 1))
        self.connect((self.blocks_nlog10_ff_0, 0), (self.rssi, 0))
        self.connect((self.blocks_selector_0, 0), (self.analog_ctcss_squelch_ff_0, 0))
        self.connect((self.blocks_selector_0, 1), (self.blocks_selector_0_0, 1))
        self.connect((self.blocks_selector_0_0, 0), (self.blocks_add_xx_0, 0))
        self.connect((self.filter_fft_low_pass_filter_0, 0), (self.blocks_nlog10_ff_0, 0))
        self.connect((self.iio_pluto_source_0, 0), (self.blocks_freqshift_cc_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "rptr")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_uri(self):
        return self.uri

    def set_uri(self, uri):
        self.uri = uri

    def get_func_rssi(self):
        return self.func_rssi

    def set_func_rssi(self, func_rssi):
        self.func_rssi = func_rssi
        self.set_pluto_gain(rpt.agc(self.func_rssi))
        self.set_rs(rpt.newrssi(self.func_rssi))

    def get_func_nbfm_busy(self):
        return self.func_nbfm_busy

    def set_func_nbfm_busy(self, func_nbfm_busy):
        self.func_nbfm_busy = func_nbfm_busy
        self.set_rx_status(rpt.status(self.func_nbfm_busy))

    def get_conf_hangtime(self):
        return self.conf_hangtime

    def set_conf_hangtime(self, conf_hangtime):
        self.conf_hangtime = conf_hangtime
        self.set_delay(self.conf_hangtime)

    def get_conf_ctcss(self):
        return self.conf_ctcss

    def set_conf_ctcss(self, conf_ctcss):
        self.conf_ctcss = conf_ctcss
        self.set_pl_freq(self.conf_ctcss)

    def get_samp_bf(self):
        return self.samp_bf

    def set_samp_bf(self, samp_bf):
        self.samp_bf = samp_bf
        self.set_samp_rate(self.samp_bf*9)
        self.band_pass_filter_0.set_taps(firdes.band_pass(1, self.samp_bf, 50, 3500, 100, window.WIN_HAMMING, 6.76))

    def get_rx_status(self):
        return self.rx_status

    def set_rx_status(self, rx_status):
        self.rx_status = rx_status
        self.set_beacon_att(rpt.beacon_att(self.rx_status))
        self.qtgui_ledindicator_0_0_0.setState(self.rx_status)

    def get_rs(self):
        return self.rs

    def set_rs(self, rs):
        self.rs = rs
        self.set_RSSI(self.rs)

    def get_pl_freq(self):
        return self.pl_freq

    def set_pl_freq(self, pl_freq):
        self.pl_freq = pl_freq
        self._conf_ctcss_config = configparser.ConfigParser()
        self._conf_ctcss_config.read('./rptr.ini')
        if not self._conf_ctcss_config.has_section('main'):
        	self._conf_ctcss_config.add_section('main')
        self._conf_ctcss_config.set('main', 'ctcss', str(self.pl_freq))
        self._conf_ctcss_config.write(open('./rptr.ini', 'w'))
        self.set_ctcss_en(rpt.ctcss_en(self.pl_freq))
        self._pl_freq_callback(self.pl_freq)
        self.analog_ctcss_squelch_ff_0.set_frequency(self.pl_freq)

    def get_func_prb_beacon_busy(self):
        return self.func_prb_beacon_busy

    def set_func_prb_beacon_busy(self, func_prb_beacon_busy):
        self.func_prb_beacon_busy = func_prb_beacon_busy
        self.set_beacon_status(rpt.status(self.func_prb_beacon_busy))

    def get_func_ctcssdec(self):
        return self.func_ctcssdec

    def set_func_ctcssdec(self, func_ctcssdec):
        self.func_ctcssdec = func_ctcssdec
        self.set_ctcss_status(rpt.status(self.func_ctcssdec))

    def get_func_bfbusy(self):
        return self.func_bfbusy

    def set_func_bfbusy(self, func_bfbusy):
        self.func_bfbusy = func_bfbusy
        self.set_ptt(rpt.hang(self.func_bfbusy, self.delay))

    def get_delay(self):
        return self.delay

    def set_delay(self, delay):
        self.delay = delay
        self._conf_hangtime_config = configparser.ConfigParser()
        self._conf_hangtime_config.read('./rptr.ini')
        if not self._conf_hangtime_config.has_section('main'):
        	self._conf_hangtime_config.add_section('main')
        self._conf_hangtime_config.set('main', 'hangtime', str(self.delay))
        self._conf_hangtime_config.write(open('./rptr.ini', 'w'))
        self.set_ptt(rpt.hang(self.func_bfbusy, self.delay))

    def get_conf_trxcal(self):
        return self.conf_trxcal

    def set_conf_trxcal(self, conf_trxcal):
        self.conf_trxcal = conf_trxcal
        self.set_oscref(self.conf_trxcal)

    def get_conf_squelch(self):
        return self.conf_squelch

    def set_conf_squelch(self, conf_squelch):
        self.conf_squelch = conf_squelch
        self.set_sq_lvl(self.conf_squelch)

    def get_conf_offset(self):
        return self.conf_offset

    def set_conf_offset(self, conf_offset):
        self.conf_offset = conf_offset
        self.set_offset(self.conf_offset)

    def get_conf_freq(self):
        return self.conf_freq

    def set_conf_freq(self, conf_freq):
        self.conf_freq = conf_freq
        self.set_freq(self.conf_freq)

    def get_sq_lvl(self):
        return self.sq_lvl

    def set_sq_lvl(self, sq_lvl):
        self.sq_lvl = sq_lvl
        self._conf_squelch_config = configparser.ConfigParser()
        self._conf_squelch_config.read('./rptr.ini')
        if not self._conf_squelch_config.has_section('main'):
        	self._conf_squelch_config.add_section('main')
        self._conf_squelch_config.set('main', 'squelch', str(self.sq_lvl))
        self._conf_squelch_config.write(open('./rptr.ini', 'w'))
        self.analog_simple_squelch_cc_0.set_threshold(self.sq_lvl+59)

    def get_shift(self):
        return self.shift

    def set_shift(self, shift):
        self.shift = shift
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.shift/self.samp_rate)
        self.iio_pluto_source_0.set_frequency((int)(self.freq)+(self.shift)+(self.oscref)+(self.offset))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.band_pass_filter_0_0.set_taps(firdes.complex_band_pass(1, self.samp_rate, -self.filter_wide, self.filter_wide, 2000, window.WIN_HAMMING, 6.76))
        self.band_pass_filter_0_0_0.set_taps(firdes.complex_band_pass(1, self.samp_rate, -self.filter_wide, self.filter_wide, 2000, window.WIN_HAMMING, 6.76))
        self.blocks_freqshift_cc_0.set_phase_inc(2.0*math.pi*self.shift/self.samp_rate)
        self.filter_fft_low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate, 50, 25, window.WIN_HAMMING, 6.76))
        self.iio_pluto_sink_0.set_samplerate(self.samp_rate)
        self.iio_pluto_source_0.set_samplerate(self.samp_rate)
        self.iio_pluto_source_0.set_filter_params('Auto', '', (float)(self.samp_rate)/(4), (float)(self.samp_rate)/(3))

    def get_ptt(self):
        return self.ptt

    def set_ptt(self, ptt):
        self.ptt = ptt
        self.analog_pwr_squelch_xx_0.set_threshold((int)(-self.ptt)*(100))
        self.qtgui_ledindicator_0_0.setState(self.ptt)

    def get_pluto_gain(self):
        return self.pluto_gain

    def set_pluto_gain(self, pluto_gain):
        self.pluto_gain = pluto_gain
        self.iio_pluto_source_0.set_gain(0, self.pluto_gain)

    def get_oscref(self):
        return self.oscref

    def set_oscref(self, oscref):
        self.oscref = oscref
        self._conf_trxcal_config = configparser.ConfigParser()
        self._conf_trxcal_config.read('./rptr.ini')
        if not self._conf_trxcal_config.has_section('main'):
        	self._conf_trxcal_config.add_section('main')
        self._conf_trxcal_config.set('main', 'trxcal', str(self.oscref))
        self._conf_trxcal_config.write(open('./rptr.ini', 'w'))
        self.iio_pluto_sink_0.set_frequency((int)(self.freq)+(self.oscref))
        self.iio_pluto_source_0.set_frequency((int)(self.freq)+(self.shift)+(self.oscref)+(self.offset))

    def get_offset(self):
        return self.offset

    def set_offset(self, offset):
        self.offset = offset
        self._conf_offset_config = configparser.ConfigParser()
        self._conf_offset_config.read('./rptr.ini')
        if not self._conf_offset_config.has_section('main'):
        	self._conf_offset_config.add_section('main')
        self._conf_offset_config.set('main', 'offset', str(self.offset))
        self._conf_offset_config.write(open('./rptr.ini', 'w'))
        self.iio_pluto_source_0.set_frequency((int)(self.freq)+(self.shift)+(self.oscref)+(self.offset))

    def get_max_dev(self):
        return self.max_dev

    def set_max_dev(self, max_dev):
        self.max_dev = max_dev
        self.analog_nbfm_rx_0.set_max_deviation(self.max_dev)
        self.analog_nbfm_tx_0.set_max_deviation(self.max_dev)

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self._conf_freq_config = configparser.ConfigParser()
        self._conf_freq_config.read('./rptr.ini')
        if not self._conf_freq_config.has_section('main'):
        	self._conf_freq_config.add_section('main')
        self._conf_freq_config.set('main', 'freq', str(self.freq))
        self._conf_freq_config.write(open('./rptr.ini', 'w'))
        self.iio_pluto_sink_0.set_frequency((int)(self.freq)+(self.oscref))
        self.iio_pluto_source_0.set_frequency((int)(self.freq)+(self.shift)+(self.oscref)+(self.offset))

    def get_filter_wide(self):
        return self.filter_wide

    def set_filter_wide(self, filter_wide):
        self.filter_wide = filter_wide
        self.band_pass_filter_0_0.set_taps(firdes.complex_band_pass(1, self.samp_rate, -self.filter_wide, self.filter_wide, 2000, window.WIN_HAMMING, 6.76))
        self.band_pass_filter_0_0_0.set_taps(firdes.complex_band_pass(1, self.samp_rate, -self.filter_wide, self.filter_wide, 2000, window.WIN_HAMMING, 6.76))

    def get_ctcss_status(self):
        return self.ctcss_status

    def set_ctcss_status(self, ctcss_status):
        self.ctcss_status = ctcss_status
        self.qtgui_ledindicator_0_0_0_0.setState(self.ctcss_status)

    def get_ctcss_en(self):
        return self.ctcss_en

    def set_ctcss_en(self, ctcss_en):
        self.ctcss_en = ctcss_en
        self.blocks_selector_0.set_output_index(self.ctcss_en)
        self.blocks_selector_0_0.set_input_index(self.ctcss_en)

    def get_buffer(self):
        return self.buffer

    def set_buffer(self, buffer):
        self.buffer = buffer

    def get_beacon_status(self):
        return self.beacon_status

    def set_beacon_status(self, beacon_status):
        self.beacon_status = beacon_status
        self.qtgui_ledindicator_0_0_0_0_0.setState(self.beacon_status)

    def get_beacon_att(self):
        return self.beacon_att

    def set_beacon_att(self, beacon_att):
        self.beacon_att = beacon_att
        self.blocks_multiply_const_vxx_0.set_k(self.beacon_att)

    def get_RSSI(self):
        return self.RSSI

    def set_RSSI(self, RSSI):
        self.RSSI = RSSI
        Qt.QMetaObject.invokeMethod(self._RSSI_label, "setText", Qt.Q_ARG("QString", str(self._RSSI_formatter(self.RSSI))))



def argument_parser():
    parser = ArgumentParser()
    parser.add_argument(
        "--uri", dest="uri", type=str, default='ip:pluto.local',
        help="Set URI [default=%(default)r]")
    return parser


def main(top_block_cls=rptr, options=None):
    if options is None:
        options = argument_parser().parse_args()

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls(uri=options.uri)

    tb.start()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
