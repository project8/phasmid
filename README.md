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

This will create an `ArtooDaq` instance which encapsulates the ROACH2 system. Some high-level methods are available to configure the system as needed, for example to tune channel *a* to 1234MHz, set the gain for that channel to 7, and to set the FFT shift vector for channels *a* and *b* to prevent overflow, do

`r2.tune_ddc_1st_to_freq(1234e6,tag='a')`

`r2.set_gain(7,tag='a')`

`r2.set_fft_shift('1101010101010',tag='ab')`.

The library also enables capturing and manipulating data packets transmitted by the ROACH2. To grab 5 packets received on socket 10.0.11.1.4001, do

`pkts = r2.grab_packets(n=5,dsoc_desc=("10.0.11.1",4001),close_soc=True)`.

This returns a list of 5 packets, each an instance of the `Packet` class which encapsulates the format which the ROACH2 uses to transmit data. A packet will contain either frequency-domain data (`pkts[0].freq_not_time` evaluates to `True`) or time-domain data (`pkts[0].freq_not_time` evaluates to `False`). For any packet, to get the sampled data do

`x = pkts[0].interpret_data()`.
