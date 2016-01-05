function [formatted_input] = stage4_packetize_init(remap_io,keep_only)

    % load registers / meta-info from stage1_ddc
    formatted_input = input_data_from_output('stage1_ddc');
    formatted_input = trim_in_out(formatted_input,{'rst','ut','T'});
    
    % load data from output of quantization stage, for time and freq data
    N_remap = size(remap_io,1);
    [remap_io_t,remap_io_f] = deal(remap_io);
    [keep_only_t,keep_only_f] = deal(keep_only);
    for ii=1:N_remap
        remap_io_t{ii,2} = ['t' remap_io{ii,2}];
        remap_io_f{ii,2} = ['f' remap_io{ii,2}];
    end
    N_keep = numel(keep_only);
    for ii=1:N_keep
        keep_only_t{ii} = ['t' keep_only{ii}];
        keep_only_f{ii} = ['f' keep_only{ii}];
    end
    fmt_input_t = input_data_from_output('stage3_quantize_time-domain');
    fmt_input_f = input_data_from_output('stage3_quantize_freq-domain');
    fmt_input_t = trim_in_out(remap_in_out(fmt_input_t,remap_io_t),keep_only_t);
    fmt_input_f = trim_in_out(remap_in_out(fmt_input_f,remap_io_f),keep_only_f);
    
    % splice together into formatted_input
    fn_t = fieldnames(fmt_input_t);
    for ii=1:numel(fn_t)
        fn = fn_t{ii};
        formatted_input.(fn) = fmt_input_t.(fn);
    end
    fn_f = fieldnames(fmt_input_f);
    for ii=1:numel(fn_f)
        fn = fn_f{ii};
        formatted_input.(fn) = fmt_input_f.(fn);
    end
end