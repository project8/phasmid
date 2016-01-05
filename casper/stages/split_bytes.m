function [w64bit] = split_bytes(uint64_val)
% split uint64 input into 8x uint8
    w64bit = uint8(zeros(1,8));
    for ii=1:8
        w64bit(ii) = mod(floor(double(uint64_val)/(256^(ii-1))),256);
    end
end