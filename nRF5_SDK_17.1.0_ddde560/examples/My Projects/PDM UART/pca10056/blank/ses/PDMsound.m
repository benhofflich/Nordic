filename = 'PDM Output7.txt';
q = quantizer('fixed', 'nearest', 'saturate', [16 0]);% quantizer object for num2hex function  
FID = fopen(filename);
dataFromfile = textscan(FID, '%s');% %s for reading string values (hexadecimal numbers)
dataFromfile = dataFromfile{1};
fclose(FID);
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
plot(decData)

bode(decData)