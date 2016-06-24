# phasmid
ROACH2-based data acquisition system.

## Development environment

### Gateware development
The FPGA gateware is developed in Matlab/Simulink on [this fork](https://github.com/sma-wideband/mlib_devel) (commit [a288ff3](https://github.com/sma-wideband/mlib_devel/commit/a288ff331e933f312ff1c73956b1096ce323a772)) of [mlib_devel](https://github.com/casper-astro/mlib_devel).

In addition it requires [acasperelle](https://github.com/anyoung/acasperelle/blob/master/README.md).

See https://casper.berkeley.edu/ for more details about setting up the CASPER toolflow.

### Software development

#### Low level monitor and control
The lowest level monitor and control library [r2daq.py](https://github.com/project8/phasmid/blob/master/software/monctrl/r2daq.py) is developed on Python and requires [adc5g (0.0.1)](https://github.com/sma-wideband/adc_tests) and [corr (0.7.3)](https://pypi.python.org/pypi/corr/0.7.3), as well as a few other standard python packages.

##### Getting started
With all dependencies installed, in an IPython shell import the interface library

`import r2daq`

and try

`r2 = r2daq.ArtooDaq('roach2_hostname',boffile='latest-build')`.

This will create an `ArtooDaq` instance which encapsulates the ROACH2 system.

##### ADC calibration

By default the system will at startup step through an ADC interface calibration procedure as well as an ADC core calibration procedure. These can be turned on / off using the flags `do_ogp_cal` (core calibration) and `do_adcif_cal` (interface calibration) as in,

`r2 = r2daq.ArtooDaq('roach2_hostname',boffile='latest-build',do_ogp_cal=False,do_adcif_cal=False)`.

The *interface calibration* is needed to ensure that the FPGA is synchronized properly with the data streamed from the ADC and should typically *not* be disabled when data needs capturing. It does save startup time though, and is useful for debugging monitor / control software.

The *core calibration* attempts to remove artefacts in the data. Without doing this there will likely appear some spurs in the spectrum, most prominently at half the Nyquist frequency. This procedure takes even more time than ADC interface calibratoin and can also be disabled for debugging, but it's recommended for actual data capturing.

##### Network configuration

The control software now allows specifying more general IP settings for the ROACH2 data network interfaces. To set IP settings for the interfaces of channels *a* and *b*, do

`cfg_a = self.make_interface_config_dictionary('192.168.10.100',4000,'192.168.10.63',4001,dest_mac='00:60:dd:44:91:e7',tag='a')`
`cfg_b = self.make_interface_config_dictionary('192.168.10.101',4000,'192.168.10.64',4001,dest_mac='00:60:dd:44:91:e8',tag='b')`
`cfg_list = [cfg_a,cfg_b]`
r2 = r2daq.ArtooDaq('roach2_hostname',boffile='latest-build',ifcfg=cfg_list)`.

Currently the IP settings can only be configured at system startup (when calling `ArtooDaq.__init__` with `boffile` not equal to `None`). Future changes in the bitcode and software will enable the system to be reconfigured in a more flexible way.

##### Channel configuration

Some high-level methods are available to configure the system as needed, for example to tune channel *a* to 1234MHz, set the gain for that channel to 7, and to set the FFT shift vector for channels *a* and *b* to prevent overflow, do

`r2.tune_ddc_1st_to_freq(1234e6,tag='a')`

`r2.set_gain(7,tag='a')`

`r2.set_fft_shift('1101010101010',tag='ab')`.

The library also enables capturing and manipulating data packets transmitted by the ROACH2. To grab 5 packets received on socket 10.0.11.1.4001, do

`pkts = r2.grab_packets(n=5,dsoc_desc=("10.0.11.1",4001),close_soc=True)`.

This returns a list of 5 packets, each an instance of the `Packet` class which encapsulates the format which the ROACH2 uses to transmit data. A packet will contain either frequency-domain data (`pkts[0].freq_not_time` evaluates to `True`) or time-domain data (`pkts[0].freq_not_time` evaluates to `False`). For any packet, to get the sampled data do

`x = pkts[0].interpret_data()`.
