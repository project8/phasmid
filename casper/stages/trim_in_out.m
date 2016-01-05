function [s_trimmed] = trim_in_out(s,keep_only)
% Trim fields in s down to those listed in keep_only, return in s_trimmed
%

    s_trimmed = struct();
    N_keep = numel(keep_only);
    for ii=1:N_keep
        keep_name = keep_only{ii};
        if isfield(s,keep_name)
            s_trimmed.(keep_name) = s.(keep_name);
        end
    end
end