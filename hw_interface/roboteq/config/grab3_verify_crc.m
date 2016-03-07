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