function [formatted_input] = stage2_ddc_init(remap_io,keep_only)
    % load data from output of stage1_ddc
    formatted_input = input_data_from_output('stage1_ddc');
    
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