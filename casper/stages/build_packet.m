function [pkt,pkt_raw_collection] = build_packet(dout_grp,tx_valid,tx_eof)
% Reconstruct packets from line data.
%
% dout_grp is the 64bit data given as 8x parallel streams of 8bit data.
% tx_valid and tx_eof are signal line data.
%
% All input should be "structure with time" format.
%
    % simplify signals
    tx_valid = tx_valid.signals.values;
    tx_eof = tx_eof.signals.values;
    D = numel(dout_grp);
    N = numel(tx_valid);
    data = zeros(N,D);
    for ii=1:D
        % convert to int8
        data(:,ii) = dout_grp{ii}.signals.values*128;
    end
    
    % count number of complete packets and initialize
    N_pkt = numel(find(tx_eof == 1));
    pkt = repmat(init_packet(),[N_pkt,1]);
    
    % for each packet, cut the data and fill structure
    idx_b = find(tx_valid == 1);
    idx_b = idx_b(tx_valid(idx_b - 1) == 0);
    idx_b = idx_b(1:N_pkt);
    idx_e = find(tx_eof == 1);
    if nargout > 1
        pkt_raw_collection = cell(N_pkt,1);
    end
    for ii=1:N_pkt
        pkt_raw = data(idx_b(ii):idx_e(ii),:);
        pkt(ii) = fill_packet(pkt_raw);
        if nargout > 1
            pkt_raw_collection{ii} = pkt_raw;
        end
    end
end

function [d_flat] = interleave_samples(d_vec)
% d_vec is an NxD array in which each column represents a single stream in
% demuxed data.
%
    [N,D] = size(d_vec);
    d_flat = zeros(D*N,1);
    for ii=1:D
        d_flat(ii:D:end) = d_vec(:,ii);
    end
end

function [pkt] = init_packet()
% Create empty packet structure
%
    pkt = struct(...
        'Unix_Time',0,...
        'Frame',0,...
        'IF_ID',0,...
        'Digital_ID',0,...
        'User_Data0',0,...
        'User_Data1',0,...
        'Reserved0',0,...
        'Reserved1',0,...
        'Type','n',...
        'Data',double(zeros(4096,1) + 1i*zeros(4096,1)) ...
    );
end

function [pkt] = fill_packet(pkt_raw)
% pkt_raw contains 1028 rows of 8x 8bit values.
%
    pkt = init_packet();
    % 1st 64bit word
    ii_64 = 1;
    pkt.IF_ID = splice_bytes(pkt_raw(ii_64,:),58,63);
    pkt.Digital_ID = splice_bytes(pkt_raw(ii_64,:),52,57);
    pkt.Frame = splice_bytes(pkt_raw(ii_64,:),32,51);
    pkt.Unix_Time = splice_bytes(pkt_raw(ii_64,:),0,31);
    % 2nd 64bit word
    ii_64 = 2;
    pkt.User_Data0 = splice_bytes(pkt_raw(ii_64,:),32,63);
    pkt.User_Data1 = splice_bytes(pkt_raw(ii_64,:),0,31);
    % 3rd 64bit word
    ii_64 = 3;
    pkt.Reserved0 = splice_bytes(pkt_raw(ii_64,:),0,63);
    % 4th 64bit word
    ii_64 = 4;
    pkt.Reserved1 = splice_bytes(pkt_raw(ii_64,:),0,63);
    if splice_bytes(pkt_raw(ii_64,:),63,63) == 1
        pkt.Type = 'f';
    else
        pkt.Type = 't';
    end
    % data words
    ii_64 = 5;
    this_data_im = pkt_raw(ii_64:end,2:2:end)';
    this_data_re = pkt_raw(ii_64:end,1:2:end)';
    pkt.Data = this_data_re(:) + 1i*this_data_im(:);
end

function [uint64_sel] = splice_bytes(w64bit,lsb,msb)
% lsb,msb are zero-based
% w64bit is given as [lsB, ...., msB]
    if numel(w64bit) < 8
        error('Need 8 bytes in 64-bit word');
    end
    
    % make sure each element is uint8
    w64bit = mod(w64bit+256,256);
    
    uint64_w64bit = uint64(0);
    for ii=1:8
        uint64_w64bit = uint64_w64bit + 256^(ii-1)*w64bit(ii);
    end
    uint64_sel = uint64(mod(uint64_w64bit,2^(msb+1)));
    uint64_sel = uint64(floor(double(uint64_sel)/2^lsb));
end