filename = 'BLE Log Codec.txt';
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

all_additions = {};
for i = 1:length(hexStrings)
    packetNums = zeros(length(hexStrings(i)));
    additions = textscan(hexStrings(i),'01-23-45-67-89-AB-CD-EF-%s');
    all_additions{i,1} = additions{1,1}{1,1};
end
all_additions_char = cell2mat(all_additions);
all_additions_str = strings(length(all_additions_char),1);
for i = 1:length(all_additions_char)
    all_additions_str(i,1) = convertCharsToStrings(all_additions_char(i,:));
end
split_additions = split(all_additions_str,'-');
q = quantizer('fixed', 'nearest', 'saturate', [8 0]);% quantizer object for num2hex function 
hexNum = hex2num(q,split_additions);
%% 
pdmData = cell2mat(hexNum);
%%

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
q = quantizer('fixed', 'nearest', 'saturate', [8 0]);% quantizer object for num2hex function 
hexNum = hex2num(q,newNums);
%% 
pdmDataMat = cell2mat(hexNum);
%%
pdmData = zeros(length(pdmDataMat)*width(pdmDataMat),1);
counter = 1;
for i = 1:length(pdmDataMat)
    for j = 1:width(pdmDataMat)
        if (j == 1)
            pdmData(counter) = pdmDataMat(i,j);
        else
            pdmData(i*j) = pdmDataMat(i,j)+pdmData(counter-1);
        end
        counter = counter + 1;
    end
end

%%
Fs = 8000;
filteredAud = pdmData/max(pdmData);

x = [1:length(pdmData)]/Fs;
%plot(x,decData)
hold on
plot(x,filteredAud,'r')
