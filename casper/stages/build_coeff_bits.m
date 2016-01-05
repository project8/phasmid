function [words] = build_coeff_bits(B,datatype,groupsize)
% Build coefficient words.
%
% [WORDS] = build_coeff_bits(B,DATATYPE,GROUPSIZE) constructs a collection
% integers that form the binary representation of the values in vector B
% according to DATATYPE. GROUPSIZE values are grouped together in each
% integer such that WORDS(1) will contain the first GROUPSIZE elements in
% B, starting with B(1) in the least-significant position.
%
% DATATYPE.WordLength x GROUPSIZE must be no greater than 32.
% 
    if (datatype.WordLength * groupsize > 32)
        error('Maximum of 32 bits allowed per group.');
    end

    s = strcmpi(datatype.Signedness,'Signed');
    w = datatype.WordLength;
    f = datatype.FractionLength;
    N = ceil(numel(B)/groupsize);
    words = uint32(zeros(N,1));
    shift_factor = 2^w;
    for bb=1:numel(B)
        gg = mod(bb-1,groupsize)+1;
        nn = ceil(bb/groupsize);
        b_fixed = fi(B(bb),s,w,f);
        b_uint32 = bin2dec(b_fixed.bin);
        words(nn) = bitor(words(nn),b_uint32*(shift_factor^(gg-1)));
    end
    
end