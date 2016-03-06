% int8 serial output verification
% Alexander T. Hypes

% close ports if previous exit was ungraceful
if exist('trans', 'var') delete(trans); end
clear all; clc; close all

% init parameters
p1='COM6';  % send port
buff=1200;    % buffer size


% init serial port
trans=serial(p1,'BaudRate',115200,'DataBits',8,'InputBufferSize',buff, 'Terminator','CR');
set(trans,'InputBufferSize',buff); % set output buffer size
fopen(trans); % open com ports

% init time epoch
tid1 = tic;
tid2 = tic;
q = 0; % serial write latch
j=0;
pathfinder = false; 
go_move = 0;
spd=500;
k=0;
enc1_total = 0;
enc1prev=0;
enc2_total = 0;
enc2prev=0;
enc3_total = 0;
enc3prev=0;

int32max = 2^31-1;
int32min = -2^31;

%while 1
for i=1:1000  % base loop
    
    if go_move
        q = q + 1;
        if q > 50
            spd = -spd;
            q=0;
        end
        
        str1 = '!G 1 1000';
        %str1 = [str1 num2str(spd+randi([-100,100],1)*10)];
        fprintf(trans, str1);
        
%         fprintf('>>   ')
%         fprintf(str1)
%         fprintf('\t')        
        
        str2 = '!G 2 -1000';
        %str2 = [str2 num2str(-spd+randi([-100,100],1)*3)];
        fprintf(trans, str2);
        
%         fprintf('>>   ')
%         fprintf(str2)
%         fprintf('\t')
        
        if pathfinder==false
            str3 = '!G 3 900';
            %str3 = [str3 num2str(-spd+randi([-100,100],1)*3)];
            fprintf(trans, str3);

%             fprintf('>>   ')
%             fprintf(str3)
%             fprintf('\t')
        end
    end
    
    % << read serial data >>
    [enc1, enc2, enc3, clk, ticker, bvolts, amp1, amp2, amp3, csum, csum_, packet_rec, pbad] = enc3_read_crc(trans);
    if pbad
        j = j + 1;
    else
        diff1 = (enc1-enc1prev);
        difff1 = 0;
        if abs(diff1) > 10000
           if enc1prev > 30783
               difff1 = 61566;
           elseif enc1prev < 30784
               difff1 = -61566;
           end
        end

        diff2 = (enc2-enc2prev);
        difff2 = 0;
        if abs(diff2) > 10000
           if enc2prev > 30783
               difff2 = 61566;
           elseif enc2prev < 30784
               difff2 = -61566;
           end
        end
        
        diff3 = (enc3-enc3prev);
        difff3 = 0;
        if abs(diff3) > 10000
           if enc3prev > 30783
               difff3 = 61566;
           elseif enc3prev < 30784
               difff3 = -61566;
           end
        end        
        
        enc1_total = (enc1-enc1prev + difff1) + enc1_total;
        enc1prev=enc1;
        enc2_total = (enc2-enc2prev + difff2) + enc2_total;
        enc2prev=enc2;
        enc3_total = (enc3-enc3prev + difff3) + enc3_total;
        enc3prev=enc3;
        
        if enc1_total > int32max
            enc1_total  = enc1_total - 2^32;
        elseif enc1_total < int32min
            enc1_total = enc1_total + 2^32;
        end

        if enc2_total > int32max
            enc2_total  = enc2_total - 2^32;
        elseif enc2_total < int32min
            enc2_total = enc2_total + 2^32;
        end
        
        if enc3_total > int32max
            enc3_total  = enc3_total - 2^32;
        elseif enc3_total < int32min
            enc3_total = enc3_total + 2^32;
        end
        %fprintf('\nTotals >> \tEnc1: %11.0i \tEnc2: %11.0i \tEnc3: %11.0i \n', enc1_total,enc2_total, enc3_total)
    end
    if (packet_rec(1)==0)
        k = k+1;
    end
    
    fprintf('\n')
end

str2 = '!G 1 0';
fprintf(trans, str2);

str2 = '!G 2 0';
fprintf(trans, str2);

if pathfinder==false
    str2 = '!G 3 0';
    fprintf(trans, str2);
end

elapsed = toc(tid1);

fprintf('Time elapsed: %.01f seconds \t Raw Rate: %2.1f Hz \t Packet rate: %2.2f Hz \t Effective Avg Rate: %2.2f Hz \n',elapsed, i/elapsed, (i-(j+k))/elapsed, (1-(j+k)/i)*(i-(j+k))/elapsed);
fprintf('Total Packets: %i \t Bad Packets: %i \t Incomplete Packets: %i \t Packet Loss: %2.1f%%\n',i, j, k, (j+k)/i*100);

% close serial ports
delete(trans);
clear trans;

%end