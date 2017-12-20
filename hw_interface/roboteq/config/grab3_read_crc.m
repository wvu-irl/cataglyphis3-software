%{
  /*********************************************************************
  * Software License Agreement (BSD License)
  *
  * Copyright (c) 2016, WVU Interactive Robotics Laboratory
  *                       https://web.statler.wvu.edu/~irl/
  * All rights reserved.
  *
  *  Redistribution and use in source and binary forms, with or without
  *  modification, are permitted provided that the following conditions
  *  are met:
  *
  *   * Redistributions of source code must retain the above copyright
  *     notice, this list of conditions and the following disclaimer.
  *   * Redistributions in binary form must reproduce the above
  *     copyright notice, this list of conditions and the following
  *     disclaimer in the documentation and/or other materials provided
  *     with the distribution.
  *   * Neither the name of the Willow Garage nor the names of its
  *     contributors may be used to endorse or promote products derived
  *     from this software without specific prior written permission.
  *
  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  *  POSSIBILITY OF SUCH DAMAGE.
  *********************************************************************/
%}

function [scomp, rcomp, errstatus, ticker, pos_avg, clk, bvolts, amp1, amp2, amp3, pos3, crc, crc_, packet_rec, pbad] = grab3_read_crc(fid)
    % << read serial data >>
    j=1;

    scomp = 0;
    rcomp = 0;
    errstatus = 0;
    ticker = 0;
    pos_avg = 0;
    crc = 0;
    crc_ = 0;
    packet_rec=0;
    pbad=0;
    clk = 0;
    q=1;
    bvolts = 0;
    amp1 = 0;
    amp2 = 0;
    amp3 = 0;

    while j % loop trough until Az is found
        header=fread(fid,1); % grab a bit
        if header == 'A' % search for A
            header2 = fread(fid,1); % if A is found, look for Z
            if header2 == 'z'
                j=0; % found header Az, break loop
                fread(fid,1); % dump number
                break % prob unnecessary
            end
        end
    end

    while q && (fid.BytesAvailable < 14)
        pause(.0000001)
    end

    if fid.BytesAvailable >= 14
        packet_rec=uint8(0); % init vars
        for k=1:14
            packet_rec(k) = fread(fid,1, 'uint8');
        end
        byte4 = bitget(double(bitxor(packet_rec(1), 255)), 1:8);
        scomp = byte4(8);
        rcomp = byte4(7);
        errstatus = byte4(6);
        ticker = byte4(5)*16 + byte4(4)*8 + byte4(3)*4 + byte4(2)*2 + byte4(1);
        clk  = double(bitxor(packet_rec(3), 255))*2^8+double(bitxor(packet_rec(2), 255));
        pos_avg = (double(bitxor(packet_rec(4), 255)))*8-1000;

        bvolts = double(bitxor(packet_rec(5), 255))/5;
        amp1 = (double(bitxor(packet_rec(6), 255))-128)/12;
        amp2 = (double(bitxor(packet_rec(7), 255))-128)/12;
        amp3 = (double(bitxor(packet_rec(8), 255))-128)/12;
        pos3 = (double(bitxor(packet_rec(9), 255)))*8-1000;


        crc_  = crc32(packet_rec(1:9));
%        csum_ = mod(sum(double(packet_rec(1:9))),256);
        crc = double(packet_rec(13))*2^24 +double(packet_rec(12))*2^16 + double(packet_rec(11))*2^8 + double(packet_rec(10));
%        csum = double(packet_rec(10));

        fprintf('scomp: %i \trcomp: %i \terrstatus: %i \tpos_avg: %5.1i \tticker: %2.1i \tbvolt: %3.1f \tamp1: %3.1f \tamp2: %3.1f \tamp3: %3.1f \tcrc_: %3.1i \tcrc: %3.1i', scomp, rcomp, errstatus, pos_avg, ticker, bvolts, amp1, amp2, amp3, crc_, crc)
        if (crc ~= crc_)
            fprintf('\t\t!>>bad')
            pbad = 1;
        end
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
