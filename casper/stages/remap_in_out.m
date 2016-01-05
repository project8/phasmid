function [s_remapped] = remap_in_out(s,remap_io)
% Remap fieldnames in s from remap{ii,1} to remap{ii,2} in s_remapped.
%

    s_remapped = s;
    N_map = size(remap_io,1);
    for ii=1:N_map
        old_name = remap_io{ii,1};
        new_name = remap_io{ii,2};
        if isfield(s,old_name)
            s_remapped.(new_name) = s.(old_name);
        end
    end
    
end