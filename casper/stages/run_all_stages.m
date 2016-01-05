close all
clear all
clc

tic_start_all = tic;

% ADC -> 1st stage DDC -> Gain
run_stage1

% FFT | 2nd stage DDC
run_stage2

% Quantization
run_stage3

% Packetization
run_stage4

tic_stop_all = toc(tic_start_all);
fprintf(1,'Total simulation time is %10.2fs\n',tic_stop_all);