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
                j=0; % found header Az, break loop
                fread(fid,1); % dump ident number
                break % prob unnecessary
            end
        end
    end

    while q && (fid.BytesAvailable < 20)
        pause(.0000001)
    end

    if fid.BytesAvailable >= 20
        packet_rec=uint8(0); % init vars
        for k=1:20
            packet_rec(k) = fread(fid,1, 'uint8');
        end
        
        % calculate checksum (before data uninversion)
        if bitand(packet_rec(20),1); packet_rec(16)=0; end
        if bitand(packet_rec(20),2); packet_rec(17)=0; end
        if bitand(packet_rec(20),4); packet_rec(18)=0; end
        if bitand(packet_rec(20),8); packet_rec(19)=0; end
        
        crc_  = crc32(packet_rec(1:15));
        crc = double(packet_rec(19))*2^24 +double(packet_rec(18))*2^16 + double(packet_rec(17))*2^8 + double(packet_rec(16));        
        
        % uninvert bytes as needed
        if bitand(packet_rec(14), 1); packet_rec(1) =0; end
        if bitand(packet_rec(14), 2); packet_rec(2) =0; end
        if bitand(packet_rec(14), 4); packet_rec(3) =0; end
        if bitand(packet_rec(14), 8); packet_rec(4) =0; end
        if bitand(packet_rec(14),16); packet_rec(5) =0; end
        if bitand(packet_rec(14),32); packet_rec(6) =0; end
        if bitand(packet_rec(14),64); packet_rec(7) =0; end            
        
        if bitand(packet_rec(15), 1); packet_rec(8) =0; end
        if bitand(packet_rec(15), 2); packet_rec(9) =0; end
        if bitand(packet_rec(15), 4); packet_rec(10)=0; end
        if bitand(packet_rec(15), 8); packet_rec(11)=0; end
        if bitand(packet_rec(15),16); packet_rec(12)=0; end
        if bitand(packet_rec(15),32); packet_rec(13)=0; end
        
        % decode data
        ticker = double(packet_rec(1));
        clk  = double(packet_rec(3))*2^8+double(packet_rec(2));
        enc1 = double(packet_rec(5))*2^8+double(packet_rec(4));
        enc2 = double(packet_rec(7))*2^8+double(packet_rec(6));
        enc3 = double(packet_rec(9))*2^8+double(packet_rec(8));
        
        bvolts = double(packet_rec(10))/5;
        amp1 = (double(packet_rec(11))-128)/12;
        amp2 = (double(packet_rec(12))-128)/12;
        amp3 = (double(packet_rec(13))-128)/12;
              
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