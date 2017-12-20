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

% int8 serial output verification
% Alexander T. Hypes

% close ports if previous exit was ungraceful
if exist('trans', 'var') delete(trans); end
clear all; clc; close all

% init parameters
p1='COM4';  % send port
buff=14;    %  buffer size

% init serial port
trans=serial(p1,'BaudRate',115200,'DataBits',8,'InputBufferSize',buff, 'Terminator','CR');
set(trans,'InputBufferSize',buff); % set output buffer size
fopen(trans); % open com ports

% init time epoch
tid1 = tic;
tid2 = tic;
q = 0; % serial write latch
j=0;
k=0;

%while 1
for i=1:1000  % base loop

    % << read serial data >>
    [scomp, rcomp, errstatus, ticker, pos_avg, clk, bvolts, amp1, amp2, amp3, pos3, csum, csum_, packet_rec, pbad] = grab3_read_crc(trans);
    if pbad
        j = j + 1;
    end
    if (packet_rec(1)==0)
        k = k+1;
    end
    fprintf('\n')
end

elapsed = toc(tid1);

%fprintf('Time elapsed: %.01f seconds \n Packet rate: %2.2f Hz \n',elapsed, i/elapsed);
%fprintf('Bad Packets %i \n',j);

fprintf('Time elapsed: %.01f seconds \t Packet rate: %2.2f Hz \t Packet Effective Rate (avg): %2.2f Hz \n',elapsed, (i-(j+k))/elapsed, (1-(j+k)/i)*(i-(j+k))/elapsed);
fprintf('Total Packets: %i \t Bad Packets: %i \t Incomplete Packets: %i \t Packet Loss: %2.1f%%\n',i, j, k, (j+k)/i*100);


% close serial ports
delete(trans);
clear trans;

%end
