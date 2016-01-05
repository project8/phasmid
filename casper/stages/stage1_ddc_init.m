function [formatted_input] = stage1_ddc_init()
% Initialize input to input_core.slx
%

    % simulation run-time
    T = 32768; % run until this number of clock cycles
    D = 16; % input demux factor
    dt = 1/D; % time step for all input data
    fs = D; % sample rate
    t = (0:dt:T-dt)'; % time-vector
    N = numel(t); % number of time samples

    % downconversion parameters
    Fsamp = 3200; % "real" sampling frequency
    Fsignal = 1081; % arbitrary center frequency for signal-of-interest
    filter_order = 127; % order of FIR filter in 1st DDC stage
    % if signal-of-interest is below Fs/4, 
    if Fsignal < Fsamp/4
        ch = ceil((Fsignal-100)/100) + 2;
        if (mod(ch,2) ~= 1)
            ch = ch+1;
        end
    else
        ch = floor((Fsignal-100)/100) - 2;
        if (mod(ch,2) ~= 1)
            ch = ch-1;
        end
    end
    Fch = 150+ch*100;
    Ftrans = Fch - Fsignal;
    if Ftrans < 0
        Ftrans = Ftrans + Fsamp;
    end
    Fpass = Fch+40*[-1,1];
    Fstop = Fch+60*[-1,1];
    Wsignal = Fsignal/(Fsamp/2);
    Wpass = Fpass/(Fsamp/2);
    BWpassW = diff(Wpass);
    Wstop = Fstop/(Fsamp/2);input_core_init
    % filter coefficients
    B = firls(filter_order,[0,Wstop(1),Wpass,Wstop(2),1],[0,0,1,1,0,0]);
    B = B/(2*max(abs(B)));
    coeff_words = build_coeff_bits(B,fixdt(1,8,7),4);
    cb0 = coeff_words(4:-1:1);
    cb1 = coeff_words(8:-1:5);
    cb2 = coeff_words(12:-1:9);
    cb3 = coeff_words(16:-1:13);
    cb4 = coeff_words(20:-1:17);
    cb5 = coeff_words(24:-1:21);
    cb6 = coeff_words(28:-1:25);
    cb7 = coeff_words(32:-1:29);
    % direct digital synthesizer inputs
    dphi = fi(Ftrans/Fsamp*(2^10),0,10,0); dphi = dphi.data;
    Dphi = fi(mod(dphi*D,2^10),0,10,0); Dphi = Dphi.data;
    phi0 = fi(2^10/4,0,10,0); phi0 = phi0.data;
   
    % master control
    t0_reset = 3*D;
    t1_reset = t0_reset + 6*D;
    x_reset = [zeros(t0_reset,1); ones(t1_reset-t0_reset,1); zeros(N-t1_reset,1)];
    master_ctrl = make_signal(t, x_reset);
    
    % gain control
    gain_a = fi(6,1,8,4);
    gain_b = fi(0.3,1,8,4);
    gain_c = fi(2.0,1,8,4);
    gain_d = fi(0.5,1,8,4);
    gain_32bit_bin = [gain_d.bin,gain_c.bin,gain_b.bin,gain_a.bin];
    x_gain = bin2dec(gain_32bit_bin)*ones(N,1);
    gain_ctrl = make_signal(t, x_gain);
    
    % fft control
    shift_ab = bin2dec('1101010101011');
    shift_cd = bin2dec('1101010101011');
    x_shift = (shift_cd*(2^13) + shift_ab)*ones(N,1);
    fft_ctrl = make_signal(t, x_shift);
    
    % unix time
    x_ut0 = 12345*ones(N,1);
    unix_time0 = make_signal(t, x_ut0);

    % signal input if0
    Pn = 0.01;
    Ps = 0.1;
    x_if0 = make_if0_signal(t,Wsignal*fs/2,BWpassW*fs/2,Pn,Ps);
    if0_data = make_signal(t, x_if0);
    
    % external sync
    t0_sync = t1_reset + 2*D;
    t1_sync = t0_sync + 3*D;
    x_sync = [zeros(t0_sync,1); ones(t1_sync-t0_sync,1); zeros(N-t1_sync,1)];
    ext_sync = make_signal(t,x_sync);
    
    % format input structure
    formatted_input = struct(...
        'T',T,...
        'cb0',cb0,...
        'cb1',cb1,...
        'cb2',cb2,...
        'cb3',cb3,...
        'cb4',cb4,...
        'cb5',cb5,...
        'cb6',cb6,...
        'cb7',cb7,...
        'dphi',dphi,...
        'Dphi',Dphi,...
        'phi0',phi0,...
        'master_ctrl',master_ctrl,...
        'gain_ctrl',gain_ctrl,...
        'fft_ctrl',fft_ctrl,...
        'unix_time0',unix_time0,...
        'if0_data',if0_data,...
        'ext_sync',ext_sync ...
    );
end

function [x] = make_if0_signal(t,f0,BWf0,Pn,Ps)
    % noise component
    N = numel(t);
    xn = sqrt(Pn)*randn(N,1);
    % moving tone component
    xs = (linspace(1.5,0.5,numel(t))') .* (sqrt(Ps*2)*chirp(t,f0+BWf0*0.1,t(end),f0-BWf0*0.1)); % chirp, frequency sweep relative to channel center
    x = xs+xn;
    xs = xs / max(abs(x));
    xn = xn / max(abs(x));
    x = xs+xn;
end

function [sig] = make_signal(t,x)
    sig = struct(...
        'time',t(:),...
        'signals',struct(...
            'values',x(:),...
            'dimensions',1 ...
        )...
    );
end