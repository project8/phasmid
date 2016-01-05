function [formatted_input] = stage3_quantize_init(remap_io,keep_only,time_or_freq)

    src_input = 'stage2_ddc'; % other option is 'stage2_fft'
    if nargin > 2
        if strcmpi(time_or_freq,'t')
            src_input = 'stage2_ddc';
        elseif strcmpi(time_or_freq,'f');
            src_input = 'stage2_fft';
        else
            warning('Invalid input source for stage3_quantize. Using stage2_ddc.');
            src_input = 'stage2_ddc';
        end
    end
    
    % load data from output of previous stage
    formatted_input = input_data_from_output(src_input);
    
    % remap existing fields
    if nargin > 0 && ~isempty(remap_io)
        N_map = size(remap_io,1);
        for ii=1:N_map
            old_name = remap_io{ii,1};
            new_name = remap_io{ii,2};
            formatted_input.(new_name) = formatted_input.(old_name);
        end
    end
    
    % trim unnecessary fields
    if nargin > 1 && ~isempty(keep_only)
        formatted_input_old = formatted_input;
        formatted_input = struct();
        N_keep = numel(keep_only);
        for ii=1:N_keep
            formatted_input.(keep_only{ii}) = formatted_input_old.(keep_only{ii});
        end
    end
end