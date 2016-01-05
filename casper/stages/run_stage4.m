% stage 4
fprintf(1,'Running 4th stage:\n');
tic_start_4 = tic;

%% Packetize
fprintf(1,'\tstage4_packetize (a)...');
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
tic_start_4a = tic;
run_core('stage4_packetize',@(x) stage4_packetize_init(remap_io,keep_only));
tic_stop_4a = toc(tic_start_4a);
fprintf(1,' ... done in %10.2fs\n',tic_stop_4a);

%%
tic_stop_4 = toc(tic_start_4);
fprintf(1,'Total 4th stage time is %10.2fs\n',tic_stop_4);
