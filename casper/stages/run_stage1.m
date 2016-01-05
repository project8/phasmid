% stage 1
fprintf(1,'Running 1st stage:\n');

%% ADC input and 1st stage DDC + gain
fprintf(1,'\tstage1_ddc (a)...');
tic_start_1a = tic;
run_core('stage1_ddc',@stage1_ddc_init);
tic_stop_1a = toc(tic_start_1a);
fprintf(1,'... done in %10.2fs\n',tic_stop_1a);

%%
tic_stop_1 = toc(tic_start_1a);
fprintf(1,'Total 1st stage time is %10.2fs\n',tic_stop_1);