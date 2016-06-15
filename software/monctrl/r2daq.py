#!/usr/bin/env python

import adc5g
from copy import deepcopy
from corr.katcp_wrapper import FpgaClient
from datetime import datetime
import netifaces as ni
from numpy import (
	abs, 
	array, 
	ceil, 
	complex64, 
	concatenate, 
	float32, 
	floor, 
	int8, 
	pi, 
	uint32, 
	uint64, 
	uint8, 
	zeros,
	)
from scipy.signal import firwin2
from socket import socket, AF_INET, SOCK_DGRAM
from struct import unpack
from time import sleep, time

class Packet():
	"""
	Encapsulate an R2DAQ packet
	
	"""
	
	BYTES_IN_PAYLOAD = 8192
	BYTES_IN_HEADER = 32
	BYTES_IN_PACKET = BYTES_IN_PAYLOAD + BYTES_IN_HEADER
	
	@property
	def unix_time(self):
		return self._unix_time
	
	@property
	def pkt_in_batch(self):
		return self._pkt_in_batch
	
	@property
	def digital_id(self):
		return self._digital_id
	
	@property
	def if_id(self):
		return self._if_id
	
	@property
	def user_data_1(self):
		return self._user_data_1
	
	@property
	def user_data_0(self):
		return self._user_data_0
	
	@property
	def reserved_0(self):
		return self._reserved_0
	
	@property
	def reserverd_1(self):
		return self._reserved_1
	
	@property
	def freq_not_time(self):
		return self._freq_not_time
	
	@property
	def data(self):
		return self._data
	
	def __init__(self,ut=0,pktnum=0,did=0,ifid=0,ud0=0,ud1=0,res0=0,res1=0,fnt=False,data=None):
		"""
		Initialize Packet with the given attributes
		"""
		# assign attributes
		self._unix_time = ut
		self._pkt_in_batch = pktnum
		self._digital_id = did
		self._if_id = ifid
		self._user_data_0 = ud0
		self._user_data_1 = ud1
		self._reserved_0 = res0
		self._reserved_1 = res1
		self._freq_not_time = fnt
		self._data = data
	
	def interpret_data(self):
		"""
		Interprets the raw 8bit data as complex-valued Fix_8_7.
		
		Returns
		-------
		x : ndarray
		    Complex-valued array represented by the data.
		"""
		x = array(self.data[1::2] + 1j*self.data[0::2],dtype=complex64)
		return x
	
	@classmethod
	def FromByteString(cls,bytestr):
		"""
		Initialize Packet from the given byte string
		"""
		# check correct size packet
		len_bytes = len(bytestr)
		if not len_bytes == cls.BYTES_IN_PACKET:
			raise ValueError("Packet should comprise {0} bytes, but has {1} bytes".format(len_bytes,cls.BYTES_IN_PACKET))
		# unpack header
		hdr = unpack(">{0}Q".format(cls.BYTES_IN_HEADER/8),bytestr[:cls.BYTES_IN_HEADER])
		ut = uint32(hdr[0] & 0xFFFFFFFF)
		pktnum = uint32((hdr[0]>>uint32(32)) & 0xFFFFF)
		did = uint8(hdr[0]>>uint32(52) & 0x3F)
		ifid = uint8(hdr[0]>>uint32(58) & 0x3F)
		ud1 = uint32(hdr[1] & 0xFFFFFFFF)
		ud0 = uint32((hdr[1]>>uint32(32)) & 0xFFFFFFFF)
		res0 = uint64(hdr[2])
		res1 = uint64(hdr[3]&0x7FFFFFFFFFFFFFFF)
		fnt = not (hdr[3]&0x8000000000000000 == 0)
		# unpack data in 64bit mode to correct for byte-order
		data_64bit = array(unpack(">{0}Q".format(cls.BYTES_IN_PAYLOAD/8),bytestr[cls.BYTES_IN_HEADER:]),dtype=uint64)
		data = zeros(cls.BYTES_IN_PAYLOAD,dtype=int8)
		for ii in xrange(len(data_64bit)):
			for jj in xrange(8):
				data[ii*8+jj] = int8((data_64bit[ii]>>uint64(8*jj))&uint64(0xFF))
		return Packet(ut,pktnum,did,ifid,ud0,ud1,res0,res1,fnt,data)

class ArtooDaq(object):
	"""
	Encapsulate R2DAQ
	"""
	
	_TIMEOUT = 5
	
	_FPGA_CLOCK = 200e6
	_DEMUX = 16
	_ADC_SAMPLE_RATE = _FPGA_CLOCK * _DEMUX
	_DIGITAL_CHANNEL_WIDTH = 100e6
	_DIGITAL_CHANNELS = ['a','b','c','d','e','f']
	_FFT_ENGINES = ['ab','cd','ef']
	_PHASE_LOOKUP_DEPTH = 10
	
	@property
	def FPGA_CLOCK(self):
		return self._FPGA_CLOCK
	
	@property
	def DEMUX(self):
		return self._DEMUX
	
	@property
	def ADC_SAMPLE_RATE(self):
		return self._ADC_SAMPLE_RATE
	
	@property
	def DIGITAL_CHANNEL_WIDTH(self):
		return self._DIGITAL_CHANNEL_WIDTH
	
	@property
	def DIGITAL_CHANNELS(self):
		return self._DIGITAL_CHANNELS
	
	@property
	def FFT_ENGINES(self):
		return self._FFT_ENGINES
	
	@property
	def PHASE_LOOKUP_DEPTH(self):
		return self._PHASE_LOOKUP_DEPTH
	
	@property
	def implemented_digital_channels(self):
		return self._implemented_digital_channels
	
	@property
	def registers(self):
		registers = dict()
		for k in self.roach2.listdev():
			registers[k] = self.roach2.read_int(k)
		return registers
	
	@property
	def roach2(self):
		return self._roach2
	
	@property
	def version(self):
		reg = self.registers
		return (reg['rcs_lib'],reg['rcs_app'],reg['rcs_user'])
	
	def print_version(self,ver=None):
		"""
		Print detailed bitcode version information.
		
		Parameters
		----------
		ver : tuple
		    The version tuple to interpret, as returned by the 
		    ArtooDaq.version property. If None, then the tuple is first
		    obtained from the current ArtooDaq instance. Default is None.
		"""
		def _app_or_lib_to_str(v):
			b31_format = ('revision system','timestamp')
			_FORMAT_REVISION = 0
			_FORMAT_TIMESTAMP = 1
			
			b30_rtype = ('git','svn')
			_RTYPE_GIT = 0
			_RTYPE_SVN = 1
			
			b28_dirty = ('all changes in revision control','changes not in revision control')
			_DIRTY_SAVED = 0
			_DRITY_UNSAVED = 1
			
			str_out = ''
			v_format = (v & 0x80000000) >> 31
			str_out = '   Format: {0}'.format(b31_format[v_format])
			if v_format == _FORMAT_REVISION:
				v_rtype = (v & 0x40000000) >> 30
				str_out = '\n'.join([str_out,'     Type: {0}'.format(
					b30_rtype[v_rtype]
				)])
				v_dirty = (v & 0x10000000) >> 28
				str_out = '\n'.join([str_out,'    Dirty: {0}'.format(
					b28_dirty[v_dirty]
				)])
				if v_rtype == _RTYPE_GIT:
					v_hash = (v & 0x0FFFFFFF)
					str_out = '\n'.join([str_out,'     Hash: {0:07x}'.format(
						v_hash
					)])
				else:
					v_rev = (v & 0x0FFFFFFF)
					str_out = '\n'.join([str_out,'      Rev: {0:9d}'.format(
						v_rev
					)])
			else:
				v_timestamp = (v & 0x3FFFFFFF)
				str_out = '\n'.join([str_out,'     Time: {0}'.format(
					datetime.fromtimestamp(float(v_timestamp)).strftime("%F %T")
				)])
			return str_out

		if ver is None:
			ver = self.version
		vl,va,vu = ver
		str_out = 'CASPER Library'
		str_out = '\n'.join([str_out,'=============='])
		str_out = '\n'.join([str_out,_app_or_lib_to_str(vl)])
		str_out = '\n'.join([str_out,'Application'])
		str_out = '\n'.join([str_out,'==========='])
		str_out = '\n'.join([str_out,_app_or_lib_to_str(va)])
		str_out = '\n'.join([str_out,'User'])
		str_out = '\n'.join([str_out,'===='])
		str_out = '\n'.join([str_out,'  Version: {0:10d}'.format(vu)])
		print str_out
		
	def __init__(self,hostname,dsoc_desc=None,boffile=None):
		"""
		Initialize an ArtooDaq object.
		
		Parameters
		----------
		hostname : string
		    Address of the roach2 on the 1GbE (control) network.
		dsoc_desc : tuple
		    A tuple with the first element the IP address / hostname and 
		    the second element the port where data is to be received. This
		    argument, if not None, is passed directly to socket.bind(); see 
		    the documentation of that class for details. In this case a 
		    socket is opened and bound to the given address. If None, then
		    the data socket is not opened. Default is None.
		boffile : string
		    Program the device with this bitcode if not None. The special 
		    filename 'latest-build' uses the current build of the bit-code.
		    Default is None.
		"""
		# connect to roach and store local copy of FpgaClient
		r2 = FpgaClient(hostname)
		if not r2.wait_connected(self._TIMEOUT):
			raise RuntimeError("Unable to connect to ROACH2 named '{0}'".format(hostname))
		self._roach2 = r2
		# program bitcode
		if not boffile is None:
			self._start(boffile)
		# initialize some data structures
		self._ddc_1st = dict()
		for did in self.DIGITAL_CHANNELS:
			self._ddc_1st[did] = None
		# if requested, open data socket
		if not dsoc_desc is None:
			self.open_dsoc(dsoc_desc)
	
	def grab_packets(self,n=1,dsoc_desc=None,close_soc=False):
		"""
		Grab packets using open data socket.
		
		Calls to this method should only be made while a data socket is 
		open, unless a socket descriptor is provided. See open_dsoc() for 
		details.
		
		Parameters
		----------
		n : int
		    Number of packets to grab, default is 1.
		dsoc_desc : tuple
		    Socket descriptor tuple as for open_dsoc() method. If None, 
		    a data socket should already be open. Default is None.
		close_soc : boolean
		    Close socket after grabbing the given number of packets.
		"""
		if not dsoc_desc is None:
			self.open_dsoc(dsoc_desc)
		try:
			dsoc = self._data_socket
		except AttributeError:
			raise RuntimeError("No open data socket. Call open_dsoc() first.")
		pkts = []
		for ii in xrange(n):
			data = dsoc.recv(Packet.BYTES_IN_PACKET)
			pkts.append(Packet.FromByteString(data))
		if close_soc:
			self.close_dsoc()
		return pkts
	
	def open_dsoc(self,dsoc_desc):
		"""
		Open socket for data reception and bind.
		
		Parameters
		----------
		dsoc_desc: tuple
		    Tuple of IP address / hostname and port as passed to socket.bind().
		"""
		self._data_socket = socket(AF_INET,SOCK_DGRAM)
		self._data_socket.bind(dsoc_desc)
		return self._data_socket
	
	def close_dsoc(self):
		"""
		Close socket used for data reception.
		"""
		self._data_socket.close()
	
	def tune_ddc_1st_to_freq(self,f_c,tag='a'):
		"""
		Tune 1st stage DDC to the given center frequency.
		
		Parameters
		----------
		fc : float
		    Desired center frequency for the 100 MHz channel.
		tag : string
		    Tag associated with the digital channel, should be one of
		    {'a','b','c','d','e','f'}. Default is 'a'.
		"""
		self._check_valid_digital_channel(tag)
		# position the digital channel at f_c
		if f_c < self.ADC_SAMPLE_RATE/4:
			ch = ceil((f_c-self.DIGITAL_CHANNEL_WIDTH)/self.DIGITAL_CHANNEL_WIDTH) + 2
			if not ((ch % 2) == 1):
				ch = ch+1
		else:
			ch = floor((f_c-self.DIGITAL_CHANNEL_WIDTH)/self.DIGITAL_CHANNEL_WIDTH) - 2
			if not ((ch % 2) == 1):
				ch = ch-1
		f_ch = self.DIGITAL_CHANNEL_WIDTH*1.5 + ch*self.DIGITAL_CHANNEL_WIDTH
		f_lo = f_ch - f_c;
		if f_lo < 0:
			f_lo = f_lo + self.ADC_SAMPLE_RATE
		# set LO parameters
		assign_ddc1 = self._build_synth_assignments(f_lo,tag=tag)
		# design anti-aliasing FIR filter to match the channel
		B = self._built_in_filter_design(f_ch)
		assign_ddc1.update(self._build_filterbank_assignments(B,tag=tag))
		# set registers
		self._make_assignment(assign_ddc1)
		# update local config
		self._ddc_1st[tag] = {
			'f_c':f_c,
			'f_ch':f_ch,
			'f_lo':f_lo if f_lo < self.ADC_SAMPLE_RATE/2 else -self.ADC_SAMPLE_RATE+f_lo,
			'B':B
		}
	
	def read_ddc_1st_config(self,tag='a'):
		"""
		Extract configuration of 1st stage DDC.
		
		Parameters
		----------
		tag : string
		    Tag associated with the digital channel, should be one of
		    {'a','b','c','d','e','f'}. Default is 'a'.
		
		Returns
		-------
		ddc1_cfg : dict
		    Dictionary containing configuration information of 1st stage 
		    DDC.
		"""
		self._check_valid_digital_channel(tag)
		cfg = deepcopy(self._ddc_1st[tag])
		d_lo,d_phi0,d_dphi_demux,d_dphi = self._extract_synth_assignments(tag)
		d_B = self._extract_filterbank_assignments(tag)
		cfg['digital'] = {
			'f_lo': d_lo,
			'B': d_B
		}
		return cfg
	
	def set_gain(self,g=1.0,tag='a'):
		"""
		Set gain on output of 1st stage DDC.
		
		Parameters
		----------
		g : float
		    Real-valued gain to apply to output of first downconversion 
		    module. Binary representation is Fix_8_4 and the caller should 
		    ensure that the gain is within the allowable range [-8,7.9375].
		    Values outside this range will saturate. Default is 1.0.
		tag : string
		    Tag associated with the digital channel, should be one of
		    {'a','b','c','d','e','f'}. Default is 'a'.
		
		Notes
		-----
		No gain control register for channels {'e','f'} yet.
		"""
		self._check_valid_digital_channel(tag)
		idx = self.DIGITAL_CHANNELS.index(tag)
		if idx > 3:
			raise Warning("Gain control registers for channels 'e' and 'f' not implemented yet")
			return
		g_8bit = int8(g*16)
		if g_8bit > 127:
			g_8bit = 127
		if g_8bit < -128:
			g_8bit = -128
		regname = 'gain_ctrl'
		masked_val = self.registers[regname] & uint32(~(0xFF<<idx*8))
		self._make_assignment({regname: masked_val | (g_8bit<<(idx*8))})
	
	def set_fft_shift(self,shift_vec='1101010101010',tag='ab'):
		"""
		Set shift vector for FFT engine.
		
		Parameters
		----------
		shift_vec : string
		    String given in bit-format, '1's and '0's where 1 indicates 
		    right-shift at stage associated with the bit position. Only 
		    the first 13 characters in the string are used. Default  is 
		    '1101010101010' which ensures no overflow.
		tag : string
		    Tag selects the FFT engine to which the shift-vector is 
		    applied. Default is 'ab'.
		"""
		self._check_valid_fft_engine(tag)
		idx = self.FFT_ENGINES.index(tag)
		regname = 'fft_ctrl'
		s_13bit = int(shift_vec,2) & 0x1FFF
		masked_val = self.registers[regname] & uint32(~(0x1FFF<<idx*13))
		self._make_assignment({regname: masked_val | uint32(s_13bit<<(idx*13))})

	def calibrate_adc_ogp(self,zdok=0,
			oiter=10,otol=0.005,
			giter=10,gtol=0.005,
			piter=0,ptol=1.0,
			verbose=10
	):
		"""
		Attempt to match the cores within the ADC.

		Each of the four cores internal to each ADC has an offset, gain,
		phase, and a number of integrated non-linearity parameters that
		can be independently tuned. This method attempts to match the
		cores within the specified ADC by tuning a subset of these 
		parameters.

		Parameters
		----------
		zdok : int
			ZDOK slot that contains the ADC, should be 0 (default is 0).
			(Second ADC card not in bitcode)
		oiter : int
			Maximum number of iterations to fine-tune offset parameter (default
			is 10).
		otol : float
			If the absolute value of the mean of snapshot data normalized to
			the standard deviation from one core is below this value then the 
			offset-tuning is considered sufficient (default is 0.005).
		giter : int
			Maximum number of iterations to fine-tune gain parameter (default 
			is 10).
		gtol : float
			If the distance between the standard deviation of the data in one
			core is less than this fraction of the standard deviation in the data
			from core1, then the gain-tuning is considered sufficient (default
			value is 0.005).
		piter : int
			Phase calibration not yet implemented.
		ptol : float
			Phase calibration not yet implemented.
		verbose : int
			The higher the more verbose, control the amount of output to
			the screen. Default is 10 (probably the highest).
		
		Returns
		-------
		ogp : dict
		    The returned parameter is a dictionary that contains the optimal
		    settings for offset, gain and phase as solved during calibration.
		"""
		if verbose > 3:
			print "Attempting OGP-calibration for ZDOK{0}".format(zdok)
		co = self.calibrate_adc_offset(zdok=zdok,oiter=oiter,otol=otol,verbose=verbose)
		cg = self.calibrate_adc_gain(zdok=zdok,giter=giter,gtol=gtol,verbose=verbose)
		cp = self.calibrate_adc_phase(zdok=zdok,piter=piter,ptol=ptol,verbose=verbose)
		return {'offset': co, 'gain': cg, 'phase': cp}

	def calibrate_adc_offset(self,zdok=0,oiter=10,otol=0.005,verbose=10):
		"""
		Attempt to match the core offsets within the ADC.

		See ArtooDaq.calibrate_adc_ogp for more details.
		"""
		# offset controlled by float varying over [-50,50] mV with 0.4 mV resolution
		res_offset = 0.4
		lim_offset = [-50.0,50.0]
		groups = 8
		test_step = 10*res_offset
		if verbose > 3:
			print "  Offset calibration ZDOK{0}:".format(zdok)
		for ic in xrange(1,5):
			adc5g.set_spi_offset(self.roach2,zdok,ic,0)
		x1 = self._snap_per_core(zdok=zdok,groups=groups)
		sx1 = x1.std(axis=0)
		mx1 = x1.mean(axis=0)/sx1
		if verbose > 5:
			print "    ...offset: with zero-offsets, means are                                        [{0}]".format(
				", ".join(["{0:+7.4f}".format(imx) for imx in mx1])
			)
		for ic in xrange(1,5):
			adc5g.set_spi_offset(self.roach2,zdok,ic,test_step)
		x2 = self._snap_per_core(zdok=zdok,groups=groups)
		sx2 = x2.std(axis=0)
		mx2 = x2.mean(axis=0)/sx2
		if verbose > 5:
			print "    ...offset: with {0:+4.1f} mV offset, means are                                      [{1}]".format(
				test_step,
				", ".join(["{0:+7.4f}".format(imx) for imx in mx2])
			)
		d_mx = (mx2 - mx1)/test_step
		core_offsets = -mx1/d_mx
		for ic in xrange(1,5):
			adc5g.set_spi_offset(self.roach2,zdok,ic,core_offsets[ic-1])
			core_offsets[ic-1] = adc5g.get_spi_offset(self.roach2,zdok,ic)
		x = self._snap_per_core(zdok=zdok,groups=groups)
		sx = x.std(axis=0)
		mx = x.mean(axis=0)/sx
		if verbose > 5:
			print "    ...offset: solution offsets are [{0}] mV, means are [{1}]".format(
				", ".join(["{0:+6.2f}".format(ico) for ico in core_offsets]),
				", ".join(["{0:+7.4f}".format(imx) for imx in mx])
			)
		if any(abs(mx) >= otol):
			if verbose > 5:
				print "    ...offset: solution not good enough, iterating (tol={0:4.4f},iter={1:d})".format(otol,oiter)
			for ii in xrange(0,oiter):
				for ic in xrange(1,5):
					if mx[ic-1] > otol:
						adc5g.set_spi_offset(self.roach2,zdok,ic,core_offsets[ic-1]-res_offset)
					elif mx[ic-1] < -otol:
						adc5g.set_spi_offset(self.roach2,zdok,ic,core_offsets[ic-1]+res_offset)
					core_offsets[ic-1] = adc5g.get_spi_offset(self.roach2,zdok,ic)
				x = self._snap_per_core(zdok=zdok,groups=groups)
				sx = x.std(axis=0)
				mx = x.mean(axis=0)/sx
				if verbose > 7:
					print "    ...offset: solution offsets are [{0}] mV, means are [{1}]".format(
						", ".join(["{0:+6.2f}".format(ico) for ico in core_offsets]),
        		        ", ".join(["{0:+7.4f}".format(imx) for imx in mx])
					)
				if all(abs(mx) < otol):
					if verbose > 5:
						print "    ...offset: solution good enough"
					break
				if ii==oiter-1:
					if verbose > 5:
						print "    ...offset: maximum number of iterations reached, aborting"
		else:
			if verbose > 5:
				print "    ...offset: solution good enough"
		return core_offsets

	def calibrate_adc_gain(self,zdok=0,giter=10,gtol=0.005,verbose=10):
		"""
		Attempt to match the core gains within the ADC.

		See ArtooDaq.calibrate_adc_ogp for more details.
		"""
		# gain controlled by float varying over [-18%,18%] with 0.14% resolution
		res_gain = 0.14
		lim_gain = [-18.0,18.0]
		groups = 8
		test_step = 10*res_gain
		if verbose > 3:
			print "  Gain calibration ZDOK{0}:".format(zdok)
		for ic in xrange(1,5):
			adc5g.set_spi_gain(self.roach2,zdok,ic,0)
		x1 = self._snap_per_core(zdok=zdok,groups=groups)
		sx1 = x1.std(axis=0)
		s0 = sx1[0]
		sx1 = sx1/s0
		if verbose > 5:
			print "    ...gain: with zero-offsets, stds are                                    [{0}]".format(
				", ".join(["{0:+7.4f}".format(isx) for isx in sx1])
			)
		# only adjust gains for last three cores, core1 is the reference
		for ic in xrange(2,5):
			adc5g.set_spi_gain(self.roach2,zdok,ic,test_step)
		x2 = self._snap_per_core(zdok=zdok,groups=groups)
		sx2 = x2.std(axis=0)
		s0 = sx2[0]
		sx2 = sx2/s0
		if verbose > 5:
			print "    ...gain: with {0:+6.2f}% gain, stds are                                    [{1}]".format(
				test_step,
				", ".join(["{0:+7.4f}".format(isx) for isx in sx2])
			)
		d_sx = 100*(sx2 - sx1)/test_step
		# give differential for core1 a non-zero value, it won't be used anyway
		d_sx[0] = 1.0
		# gains are in units percentage
		core_gains = 100*(1.0-sx1)/d_sx
		# set core1 gain to zero
		core_gains[0] = 0
		for ic in xrange(2,5):
			adc5g.set_spi_gain(self.roach2,zdok,ic,core_gains[ic-1])
			core_gains[ic-1] = adc5g.get_spi_gain(self.roach2,zdok,ic)
		x = self._snap_per_core(zdok=zdok,groups=groups)
		sx = x.std(axis=0)
		s0 = sx[0]
		sx = sx/s0
		if verbose > 5:
			print "    ...gain: solution gains are [{0}]%, stds are [{1}]".format(
				", ".join(["{0:+6.2f}".format(ico) for ico in core_gains]),
				", ".join(["{0:+7.4f}".format(isx) for isx in sx])
			)
		if any(abs(1.0-sx) >= gtol):
			if verbose > 5:
				print "    ...gain: solution not good enough, iterating (tol={0:4.4f},iter={1:d})".format(gtol,giter)
			for ii in xrange(0,giter):
				for ic in xrange(2,5):
					if (1.0-sx[ic-1]) > gtol:
						adc5g.set_spi_gain(self.roach2,zdok,ic,core_gains[ic-1]+res_gain)
					elif (1.0-sx[ic-1]) < -gtol:
						adc5g.set_spi_gain(self.roach2,zdok,ic,core_gains[ic-1]-res_gain)
					core_gains[ic-1] = adc5g.get_spi_gain(self.roach2,zdok,ic)
				x = self._snap_per_core(zdok=zdok,groups=groups)
				sx = x.std(axis=0)
				s0 = sx[0]
				sx = sx/s0
				if verbose > 7:
					print "    ...gain: solution gains are [{0}]%, stds are [{1}]".format(
							", ".join(["{0:+6.2f}".format(ico) for ico in core_gains]),
        	                ", ".join(["{0:+7.4f}".format(isx) for isx in sx])
					)
				if all(abs(1.0-sx) < gtol):
					if verbose > 5:
						print "    ...gain: solution good enough"
					break
				if ii==giter-1:
					if verbose > 5:
						print "    ...gain: maximum number of iterations reached, aborting"
		else:
			if verbose > 5:
				print "    ...gain: solution good enough"
		return core_gains

	def calibrate_adc_phase(self,zdok=0,piter=0,ptol=1.0,verbose=10):
		"""
		Attempt to match the core phases within the ADC.
		
		See ArtooDaq.calibrate_adc_ogp for more details.
		"""
		# phase controlled by float varying over [-14,14] ps with 0.11 ps resolution
		res_phase = 0.11
		lim_phase = [-14.0,14.0]
		if verbose > 3:
			print "  Phase calibration ZDOK{0}:".format(zdok)
		core_phases = zeros(4)
		for ic in xrange(1,5):
			core_phases[ic-1] = adc5g.get_spi_phase(self.roach2,zdok,ic)
		if verbose > 5:
			print "    ...phase: tuning not implemented yet, phase parameters are [{0}]".format(
				", ".join(["{0:+06.2f}".format(icp) for icp in core_phases])
			)
		return core_phases
				
	def _snap_per_core(self,zdok=0,groups=1):
		"""
		Get a snapshot of 8-bit data per core from the ADC.

		Parameters
		----------
		zdok : int
			ID of the ZDOK slot, either 0 or 1 (default is 0)
		groups : int
			Each snapshot grabs groups*(2**16) samples per core
			(default is 1).
		
		Returns
		-------
		x : ndarray
			A (groups*(2**16), 4)-shaped array in which data along the 
			first dimension contains consecutive samples taken form the 
			same core. The data is ordered such that the index along
			the second dimension matches core-indexing in the spi-
			family of functions used to tune the core parameters.
		"""
		x = zeros((0,4))
		for ig in xrange(groups):
			grab = self.roach2.snapshot_get('snap_{0}_snapshot'.format(zdok))
        	        x_ = array(unpack('%ib' %grab['length'], grab['data']))
                	x_ = x_.reshape((x_.size/4,4))
			x = concatenate((x,x_))
		return x[:,[0,2,1,3]]
	
	def _make_assignment(self,assign_dict):
		"""
		Assign values to ROACH2 software registers.
		
		Assignments are made as roach2.write_int(key,val).
		
		Parameters
		----------
		assign_dict : dict
		    Each key in assign_dict should correspond to a valid ROACH2 
		    software register name, and each val should be int compatible.
		"""
		for key in assign_dict.keys():
			val = assign_dict[key]
			self.roach2.write_int(key,val)
	
	def _built_in_filter_design(self,f_ch):
		"""
		Design basic shape-matching 127th order FIR filter.
		
		The filter will attempt to match the following gain envelope:
		    w = [0, s1, p1, p2, s1, 1]
		    h = [0,  0,  1,  1,  0, 0]
		where
		    s1 = f_ch - DIGITAL_CHANNEL_WIDTH*0.6
		    p1 = f_ch - DIGITAL_CHANNEL_WIDTH*0.4
		    p2 = f_ch + DIGITAL_CHANNEL_WIDTH*0.4
		    s2 = f_ch + DIGITAL_CHANNEL_WIDTH*0.6,
		h is normalized to a maximum of 1, and w is the angular frequency 
		normalized to pi.
		
		Parameters
		----------
		f_ch : float
		    Center frequency of bandpass filter.
		
		Returns
		-------
		B : ndarray
		    FIR filter coefficients.
		"""
		# filter channel should be at least more than digital bandwidth from sampled boundaries
		f_lower = self.DIGITAL_CHANNEL_WIDTH
		f_upper = self.ADC_SAMPLE_RATE/2-self.DIGITAL_CHANNEL_WIDTH
		if f_ch <= f_lower or f_ch >= f_upper:
			raise RuntimeError("Digital channel center frequency is {0:7.3f}MHz, but should be within ({1:7.3f},{2:7.3f}) MHz".format(f_ch/1e6,f_lower/1e6,f_upper/1e6))
		# construct envelope
		f_pass = f_ch + array([-1,1])*self.DIGITAL_CHANNEL_WIDTH*0.4
		f_stop = f_ch + array([-1,1])*self.DIGITAL_CHANNEL_WIDTH*0.6
		w_pass = f_pass/(self.ADC_SAMPLE_RATE/2)
		w_stop = f_stop/(self.ADC_SAMPLE_RATE/2)
		filt_gain = array([0,0,1,1,0,0])
		filt_freq = concatenate(([0],[w_stop[0]], w_pass, [w_pass[1]], [1.0]))
		B = firwin2(128,filt_freq,filt_gain,window='boxcar')
		# normalize to absolute maximum of 0.5
		B = 0.5*B/(abs(B).max())
		return B
	
	def _build_synth_assignments(self,f_lo,phase_offset=0,tag='a'):
		"""
		Build phase increment words for synthesizer.
		
		Parameters
		----------
		f_lo : float
		    LO frequency.
		phase_offset : float
		    LO phase offset in radians.
		tag : string
		    Tag associated with the downconverter, should be one of
		    {'a','b','c','d','e','f'}. Default is 'a'.
		
		Returns
		-------
		assign_synth : dict
		    Each (key,val) pair can be used to assign devices as in
		    roach2.write_int(key,val)
		"""
		# calculate discretized parameter values
		phase_res = 2**self.PHASE_LOOKUP_DEPTH
		mask = phase_res-1
		dphi = uint32(f_lo/self.ADC_SAMPLE_RATE*phase_res)
		dphi_demux = uint32((dphi*self.DEMUX) % phase_res)
		phi0 = uint32(phase_offset*phase_res/(2*pi))
		# build single 30bit value
		word = ((dphi_demux&mask)<<(self.PHASE_LOOKUP_DEPTH*2)) | ((dphi&mask)<<self.PHASE_LOOKUP_DEPTH) | (phi0&mask)
		assign_synth = {'ddc_1st_' + tag + '_synth_input_dphi':word}
		return assign_synth
	
	def _extract_synth_assignments(self,tag='a'):
		"""
		Extract synthesizer settings from the contents of the control registers.
		
		Parameters
		----------
		tag : string
		    Tag associated with the downconverter, should be one of
		    {'a','b','c','d','e','f'}. Default is 'a'.
		
		Returns
		-------
		f_lo : float
		    Digital local oscillator frequency in Hz. If this value is 
		    equal to half the sampling rate it is negated, and for larger 
		    values it wraps toward -0.
		phi0 : float
		    Phase offset in radians.
		dphi_demux : float
		    Phase step per demux-group of samples in radians.
		dphi : float
		    Phase step per sample in radians.
		"""
		phase_res = 2**self.PHASE_LOOKUP_DEPTH
		mask = phase_res-1
		word = self.roach2.read_int('ddc_1st_' + tag + '_synth_input_dphi')
		dphi_demux = float32((word>>(self.PHASE_LOOKUP_DEPTH*2))&mask) / phase_res * 2*pi
		dphi = float32((word>>self.PHASE_LOOKUP_DEPTH)&mask) / phase_res * 2*pi
		phi0 = float32(word&mask) / phase_res * 2*pi
		f_lo = self.ADC_SAMPLE_RATE * dphi/(2*pi)
		if f_lo >= self.ADC_SAMPLE_RATE/2:
			f_lo = -self.ADC_SAMPLE_RATE + f_lo
		return f_lo, phi0, dphi_demux, dphi

	def _build_filterbank_assignments(self,B,tag='a'):
		"""
		Build FIR filter words for anti-aliasing filter.
		
		Parameters
		----------
		B : ndarray
		    Filter coefficients for 127th order filter, or 128 coefficients 
		    in total. These values are discretized as Fix_8_7, or 8-bit 
		    fixed-point with a single integer bit. The range of values that 
		    can be represented is [-1.0,0.9921875] and it is up to the 
		    caller to ensure that the values of all filter coefficients 
		    are properly distributed.
		tag : string
		    Tag associated with the downconverter, should be one of
		    {'a','b','c','d','e','f'}. Default is 'a'.
		
		Returns
		-------
		assign_filter : dict
		    Each (key,val) pair can be used to assign devices as in
		    roach2.write_int(key,val)
		"""
		s = True #signed
		w = 8# word length
		f = 7# fraction length
		groupsize = 4#number of coefficients per 32bit word
		shift_factor = uint32(2**w)
		mask = uint32(shift_factor-1)
		# scale B to absolute maximum of 128
		B = B[:128]*128
		N = len(B);
		if not N ==128:
			raise RuntimeError("There should be 128 filter coefficients for 1st stage DDC, but {0} received".format(N))
		N_w = N/groupsize;
		words = zeros(N_w,uint32)
		for bb in xrange(N):
			gg = bb % groupsize
			nn = floor(bb/groupsize)
			b_uint32 = uint32(B[bb])
			words[nn] = words[nn] | ((b_uint32&mask)<<(w*gg))
		# setup assignment dictionary
		assign_filter = {}
		base = 'ddc_1st_' + tag
		for cb_id in xrange(8):
			this_cb = base + '_cb{0}'.format(cb_id)
			for g_id in xrange(4):
				this_key = this_cb + '_g{0}'.format(g_id)
				nn = 4*cb_id + g_id
				this_val = words[nn]
				assign_filter[this_key] = this_val
		return assign_filter
	
	def _extract_filterbank_assignments(self,tag='a'):
		"""
		Extract FIR filter coefficients from the contents of the control registers.
		
		Parameters
		----------
		tag : string
		    Tag associated with the downconverter, should be one of
		    {'a','b','c','d','e','f'}. Default is 'a'.
		
		Returns
		-------
		B : ndarray
		    FIR filter coefficients.
		"""
		s = True #signed
		w = 8# word length
		f = 7# fraction length
		groupsize = 4#number of coefficients per 32bit word
		shift_factor = uint32(2**w)
		mask = uint32(shift_factor-1)
		N = 128
		N_w = N/groupsize
		# read uint32 words
		words = zeros(N_w,uint32)
		base = 'ddc_1st_' + tag
		for cb_id in xrange(8):
			this_cb = base + '_cb{0}'.format(cb_id)
			for g_id in xrange(4):
				this_key = this_cb + '_g{0}'.format(g_id)
				nn = 4*cb_id + g_id
				words[nn] = self.roach2.read_int(this_key)
		# decompose into filter coefficients
		B = zeros(N,float32)
		for bb in xrange(N):
			gg = bb % groupsize
			nn = floor(bb/groupsize)
			b_int8 = ((words[nn] & (mask<<(w*gg)))>>(w*gg))
			if b_int8 > 127: # fix negative values
				b_int8 = b_int8 - (1<<w)
			B[bb] = b_int8
		B = B/128.0
		return B
	
	def _check_valid_digital_channel(self,tag):
		"""
		Check if digital channel tag is valid.
		
		If the tag is invalid a RuntimeError is raised.
		
		Parameters
		----------
		tag : string
		    Digital channel tag. Valid tags are any of {'a','b','c','d','e','f'}.
		
		Returns
		-------
		None
		"""
		if not tag in self.DIGITAL_CHANNELS:
			raise RuntimeError("Invalid digital channel tag '{0}', should be one of {1}".format(tag,self.DIGITAL_CHANNELS))
		if not tag in self.implemented_digital_channels:
			raise Warning("Digital channels '{0}' not implemented in bitcode".format(tag))
	
	def _check_valid_fft_engine(self,tag):
		"""
		Check if FFT engine tag is valid.
		
		If the tag is invalid a RuntimeError is raised.
		
		Parameters
		----------
		tag : string
		    FFT engine tag. Valid tags are any of {'ab','cd','ef'}.
		
		Returns
		-------
		None
		"""
		if not tag in self.FFT_ENGINES:
			raise RuntimeError("Invalid FFT engine tag '{0}', should be one of {1}".format(tag,self.FFT_ENGINES))
		if self.FFT_ENGINES.index(tag) > 1:
			raise Warning("FFT engine 'ef' not yet implemented in bitcode")
	
	def _start(self,boffile='latest-build',do_cal=True,iface="p11p1",verbose=10):
		"""
		Program bitcode on device.
		
		Parameters
		----------
		boffile : string
		    Filename of the bitcode to program. If 'latest-build' then 
		    use the current build. Default is 'latest-build'.
		do_cal : bool
		    If true then do ADC core calibration. Default is True.
		iface : string
		    Network interface connected to the data network.
		verbose : int
		    The higher the more verbose, control the amount of output to
		    the screen. Default is 10 (probably the highest).
		Returns
		-------
		"""
		
		if boffile == "latest-build":
			boffile = "r2daq_2016_May_18_1148.bof"
				
		# program bitcode
		self.roach2.progdev(boffile)
		self.roach2.wait_connected()
		if verbose > 1:
			print "Bitcode '", boffile, "' programmed successfully"
		
		# display clock speed
		if verbose > 3:
			print "Board clock is ", self.roach2.est_brd_clk(), "MHz"
		
		# ADC interface calibration
		if verbose > 3:
			print "Performing ADC interface calibration... (only doing ZDOK0)"
		adc5g.set_test_mode(self.roach2, 0)
		#~ adc5g.set_test_mode(self.roach2, 1) #<<---- ZDOK1 not yet in bitcode
		adc5g.sync_adc(self.roach2)
		opt0, glitches0 = adc5g.calibrate_mmcm_phase(self.roach2, 0, ['snap_0_snapshot',])
		#~ opt1, glitches1 = adc5g.calibrate_mmcm_phase(self.roach2, 1, ['zdok_1_snap_data',]) #<<---- ZDOK1 not yet in bitcode
		adc5g.unset_test_mode(self.roach2, 0)
		#~ adc5g.unset_test_mode(self.roach2, 1) #<<---- ZDOK1 not yet in bitcode
		if verbose > 3:
			print "...ADC interface calibration done."
		if verbose > 5:
			print "if0: opt0 = ",opt0, ", glitches0 = \n", array(glitches0)
			#~ print "if1: ",opt0, glitches0  #<<---- ZDOK1 not yet in bitcode
	
		# ADC core calibration
		if do_cal:
			self.calibrate_adc_ogp(zdok=0,verbose=verbose)
		
		# build channel-list
		ch_list = ['a','b','c','d','e','f']
		self._implemented_digital_channels = []
		for ch in ch_list:
			try:
				self.roach2.read_int("tengbe_{0}_ctrl".format(ch))
				self._implemented_digital_channels.append(ch)
			except RuntimeError:
				pass
		if verbose > 3:
			print "Valid channels in this build: {0}".format(self.implemented_digital_channels)

		# hold master reset signal and arm the manual sync
		self.roach2.write_int('master_ctrl',0x00000001 | 0x00000002)
		master_ctrl = self.roach2.read_int('master_ctrl')
		# hold 10gbe reset signal
		for ch in self.implemented_digital_channels:
			self.roach2.write_int('tengbe_{0}_ctrl'.format(ch),0x80000000)
		# ip, port of data interface on receive side
		dest_ip_str_cmp = ni.ifaddresses(iface)[2][0]['addr'].split('.')
		ip3 = int(dest_ip_str_cmp[0])
		ip2 = int(dest_ip_str_cmp[1])
		ip1 = int(dest_ip_str_cmp[2])
		ip0 = int(dest_ip_str_cmp[3])
		dest_ip = (ip3<<24) + (ip2<<16) + (ip1<<8) + ip0
		dest_port = 4001
		# fill arp table on ROACH2
		mac_iface = ni.ifaddresses(iface)[17][0]['addr']
		hex_iface = int(mac_iface.translate(None,':'),16)
		arp = [0xffffffffffff] * 256
		arp[ip0] = hex_iface
		# and configure
		ch_offset = 0
		for ch in self.implemented_digital_channels:
			# ip, port, mac of data interface on transmit side
			src_ip = (ip3<<24) + (ip2<<16) + (ip1<<8) + 2+ch_offset
			src_port = 4000
			src_mac = (2<<40) + (2<<32) + src_ip
			self.roach2.config_10gbe_core('tengbe_{0}_core'.format(ch),src_mac,src_ip,src_port,arp)
			self.roach2.write_int('tengbe_{0}_ip'.format(ch),dest_ip)
			self.roach2.write_int('tengbe_{0}_port'.format(ch),dest_port+ch_offset)
			ch_offset = ch_offset + 1
		# and release reset
		for ch in self.implemented_digital_channels:
			self.roach2.write_int('tengbe_{0}_ctrl'.format(ch),0x00000000)
		# set time, wait until just before a second boundary
		while(abs(datetime.utcnow().microsecond-9e5)>1e3):
			sleep(0.001)
		# when the system starts running it will be the next second
		ut0 = int(time())+1
		self.roach2.write_int('unix_time0',ut0)
		# release master reset signal
		master_ctrl = self.roach2.read_int('master_ctrl')
		master_ctrl = master_ctrl & 0xFFFFFFFC
		self.roach2.write_int('master_ctrl',master_ctrl)
#		# wait 100ms and then trigger start of manual sync
#		sleep(0.1)
#		master_ctrl = self.roach2.read_int('master_ctrl')
#		master_ctrl = master_ctrl & 0xFFFFFFFC
#		self.roach2.write_int('master_ctrl',master_ctrl)
		# on the 
		if verbose > 1:
			print "Configuration done, system should be running"
		#~ if verbose > 3:
			#~ print "Hard-reset for buffer overflow error"
		#~ master_ctrl = self.roach2.write_int('master_ctrl',0x00000001)
		#~ self.roach2.write_int('tengbe_a_ctrl',0x80000000)
		#~ sleep(1)
		#~ self.roach2.write_int('tengbe_a_ctrl',0x00000000)
		#~ master_ctrl = self.roach2.write_int('master_ctrl',0x00000000)
	
