filename = '3-9-22.txt';
q = quantizer('fixed', 'nearest', 'saturate', [16 0]);% quantizer object for num2hex function  
FID = fopen(filename);
dataFromfile = textscan(FID, '%s');% %s for reading string values (hexadecimal numbers)
dataFromfile = dataFromfile{1};
fclose(FID);
Fs = 6000;
%%
dataFromfile = dataFromfile(2:end-1);
newHex = strings(length(dataFromfile)/2,1);
for i = 1:length(dataFromfile)/2
    newHex(i) = [dataFromfile{2*i-1},dataFromfile{2*i}];
end
%% 
decData = hex2num(q, newHex);
%% 

%pdmData = decData(2:2:end);
decData = cell2mat(decData);

pspectrum(decData)
%audfilt = designfilt('highpassfir','PassbandFrequency',1500,'StopbandFrequency',1,'PassbandRipple',1,'StopbandAttenuation',60,'SampleRate',Fs);
%filteredAud = filtfilt(audfilt,decData);
%filteredAud = filteredAud/max(filteredAud);
filteredAud = decData/max(decData);

x = [1:length(decData)]/Fs;
plot(x,decData)
hold on
plot(x,filteredAud)
