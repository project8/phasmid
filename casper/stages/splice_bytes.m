function [uint64_sel] = splice_bytes(w64bit,lsb,msb)
% lsb,msb are zero-based
% w64bit is given as [lsB, ...., msB]
    if numel(w64bit) < 8
        error('Need 8 bytes in 64-bit word');
    end
    
    uint64_w64bit = uint64(0);
    for ii=1:8
        uint64_w64bit = uint64_w64bit + 256^(ii-1)*w64bit(ii);
    end
    uint64_sel = uint64(mod(uint64_w64bit,2^(msb+1)));
    uint64_sel = uint64(floor(double(uint64_sel)/2^lsb));
end