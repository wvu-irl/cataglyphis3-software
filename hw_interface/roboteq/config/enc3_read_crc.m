function [enc1, enc2, enc3, clk, ticker, bvolts, amp1, amp2, amp3, crc, crc_, packet_rec, pbad] = enc3_read_crc(fid)
    % << read serial data >>
    j=1;

    enc1 = 0;
    enc2 = 0;
    enc3 = 0;
    clk  = 0;
    crc = 0;
    crc_ = 0;
    packet_rec=0;
    ticker = 0;
    pbad=0;
    q=1;

    while j % loop trough until Az is found
        header=fread(fid,1); % grab a bit
        if header == 'A' % search for A
            header2 = fread(fid,1); % if A is found, look for Z
            if header2 == 'z'
                j=0; % found header AZ, break loop
                fread(fid,1); % dump ident number
                break % prob unnecessary
            end
        end
    end

    while q && (fid.BytesAvailable < 18)
        pause(.0000001)
    end

    if fid.BytesAvailable >= 18
        packet_rec=uint8(0); % init vars
        for k=1:18
            packet_rec(k) = fread(fid,1, 'uint8');
        end
        ticker = double(bitxor(packet_rec(1), 255));
        clk  = double(bitxor(packet_rec(3), 255))*2^8+double(bitxor(packet_rec(2), 255));
        enc1 = double(bitxor(packet_rec(5), 255))*2^8+double(bitxor(packet_rec(4), 255));
        enc2 = double(bitxor(packet_rec(7), 255))*2^8+double(bitxor(packet_rec(6), 255));
        enc3 = double(bitxor(packet_rec(9), 255))*2^8+double(bitxor(packet_rec(8), 255));
        
        bvolts = double(bitxor(packet_rec(10), 255))/5;
        amp1 = (double(bitxor(packet_rec(11), 255))-128)/12;
        amp2 = (double(bitxor(packet_rec(12), 255))-128)/12;
        amp3 = (double(bitxor(packet_rec(13), 255))-128)/12;
        
        
        if bitand(packet_rec(18),1) packet_rec(14)=0; end
        if bitand(packet_rec(18),2) packet_rec(15)=0; end
        if bitand(packet_rec(18),4) packet_rec(16)=0; end
        if bitand(packet_rec(18),8) packet_rec(17)=0; end
        
        crc_  = crc32(packet_rec(1:13));
%        csum_ = mod(sum(double(packet_rec(1:13))),256);
        crc = double(packet_rec(17))*2^24 +double(packet_rec(16))*2^16 + double(packet_rec(15))*2^8 + double(packet_rec(14));
%        csum = double(packet_rec(14));
      
        fprintf('enc1: %5.1i \tenc2: %5.1i \tenc3: %5.1i \tctr: %3.1i \tclk: %3.1i \tbvolt: %3.1f \tamp1: %3.1f \tamp2: %3.1f \tamp3: %3.1f \tcrc_: %3.0i \tcrc: %3.0i', enc1, enc2, enc3, ticker, clk, bvolts, amp1, amp2, amp3, crc_, crc)
        if (crc ~= crc_)
            fprintf('\t\t!>>bad')
            pbad = 1;
        end
        q=0;
    else
        fprintf('\t\t!>>incomplete')
    end
    
end

function crc = crc32(data)

%crc32   Computes the CRC-32 checksum value of a byte vector.
%--------------------------------------------------------------------------
%   CRC = crc32(DATA) computes the CRC-32 checksum value of the data stored
%   in vector DATA. The elements of DATA are interpreted as unsigned bytes
%   (uint8). The result is an unsigned 32-bit integer (uint32). Polynomial
%   bit positions have been reversed, and the algorithm modified, in order
%   to improve performance.

%   Version:    1.00
%   Programmer: Costas Vlachos
%   Date:       23-Dec-2014

% Initialize variables
    crc  = uint32(hex2dec('FFFFFFFF'));
    poly = uint32(hex2dec('EDB88320'));
    data = uint8(data);

% Compute CRC-32 value
    for i = 1:length(data)
        crc = bitxor(crc,uint32(data(i)));
        for j = 1:8
            mask = bitcmp(bitand(crc,uint32(1)));
            if mask == intmax('uint32'), mask = 0; else mask = mask+1; end
            crc = bitxor(bitshift(crc,-1),bitand(poly,mask));
        end
    end
    crc = bitcmp(crc);
end