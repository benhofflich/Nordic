filename = '3 Part PDM buff1.txt';
q = quantizer('fixed', 'nearest', 'saturate', [16 0]);% quantizer object for num2hex function  
FID = fopen(filename);
dataFromfile1 = textscan(FID, '%s');% %s for reading string values (hexadecimal numbers)
dataFromfile1 = dataFromfile1{1};
dataFromfile1 = dataFromfile1(2:end-1);
fclose(FID);
Fs = 8000;
%% 
filename = '3 Part PDM buff2.txt';
q = quantizer('fixed', 'nearest', 'saturate', [16 0]);% quantizer object for num2hex function  
FID = fopen(filename);
dataFromfile2 = textscan(FID, '%s');% %s for reading string values (hexadecimal numbers)
dataFromfile2 = dataFromfile2{1};
dataFromfile2 = dataFromfile2(2:end-1);
fclose(FID);
%% 
filename = '3 Part PDM buff3.txt';
q = quantizer('fixed', 'nearest', 'saturate', [16 0]);% quantizer object for num2hex function  
FID = fopen(filename);
dataFromfile3 = textscan(FID, '%s');% %s for reading string values (hexadecimal numbers)
dataFromfile3 = dataFromfile3{1};
dataFromfile3 = dataFromfile3(2:end-1);
fclose(FID);
%%
dataFromfile = [dataFromfile1;dataFromfile2;dataFromfile3];
newHex = strings(length(dataFromfile)/2,1);
for i = 1:length(dataFromfile)/2
    newHex(i) = [dataFromfile{2*i-1,1}, dataFromfile{2*i,1}];
end
%% 
decData = hex2num(q, newHex);
%% 

%pdmData = decData(2:2:end);
decData = cell2mat(decData);

%pspectrum(decData)
%audfilt = designfilt('highpassfir','PassbandFrequency',1500,'StopbandFrequency',1,'PassbandRipple',1,'StopbandAttenuation',60,'SampleRate',Fs);
%filteredAud = filtfilt(audfilt,decData);
%filteredAud = filteredAud/max(filteredAud);
filteredAud = decData/max(decData);

x = [1:length(decData)]/Fs;
%plot(x,decData)
hold on
plot(x,filteredAud,'r')
