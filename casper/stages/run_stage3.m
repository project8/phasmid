% stage 3
fprintf(1,'Running 3rd stage:\n');
tic_start_3 = tic;

%% Quantize - time-domain
fprintf(1,'\tstage3_quantize (a, time)...');
remap_io = {...
    'sync_out','sync_in';
    'dout_c_1','din_c_1';
    'dout_c_2','din_c_2';
};
keep_only = {...
    'T',...
    'sync_in',...
    'din_c_1',...
    'din_c_2' ...
};
tic_start_3a = tic;
run_core('stage3_quantize',@(x) stage3_quantize_init(remap_io,keep_only,'t'),'time-domain');
tic_stop_3a = toc(tic_start_3a);
fprintf(1,' ... done in %10.2fs\n',tic_stop_3a);

%% Quantize - frequency-domain
fprintf(1,'\tstage3_quantize (a, freq)...');
remap_io = {...
    'sync_out','sync_in';
    'dout0_c_1','din_c_1';
    'dout0_c_2','din_c_2';
};
keep_only = {...
    'T',...
    'sync_in',...
    'din_c_1',...
    'din_c_2' ...
};
tic_start_3a = tic;
run_core('stage3_quantize',@(x) stage3_quantize_init(remap_io,keep_only,'f'),'freq-domain');
tic_stop_3a = toc(tic_start_3a);
fprintf(1,' ... done in %10.2fs\n',tic_stop_3a);

%%
tic_stop_3 = toc(tic_start_3);
fprintf(1,'Total 3rd stage time is %10.2fs\n',tic_stop_3);
