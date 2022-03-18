filename = 'BLE Log3.txt';
fileID = fopen(filename);
line = fgetl(fileID);
hexes = {};
i = 1;
while ischar(line)
    C = textscan(line,'%s INFO Attribute value changed, handle: 0x0F, value (0x): %s');
    if (~isempty(C{1,2}))
        hexes{i,1} = C{1,2}{1,1};
        i = i+1;
    end
    line = fgetl(fileID);
end
fclose(fileID);
hexStrings = convertCharsToStrings(hexes);

singleString = "";

for i = 1:length(hexStrings)
    singleString = singleString + hexStrings(i) + "-";
end

spacedString = strrep(singleString,'-',' ');
hexNum = split(spacedString);
newNums = strings;
for i = 2:2:length(hexNum)
    newNums(((i)/2),1) = append(hexNum(i-1),hexNum(i));
end
%%
q = quantizer('fixed', 'nearest', 'saturate', [16 0]);% quantizer object for num2hex function 
hexNum = hex2num(q,newNums);
%% 
pdmData = cell2mat(hexNum);
%%
Fs = 8000;
filteredAud = pdmData/max(pdmData);

x = [1:length(pdmData)]/Fs;
%plot(x,decData)
hold on
plot(x,filteredAud,'r')
