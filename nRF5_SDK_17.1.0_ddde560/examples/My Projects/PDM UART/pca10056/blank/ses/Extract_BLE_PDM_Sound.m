clear
clc

filename = "PDM BLE Test 3-28-22.txt";
fid = fopen(filename);
formatSpec = '%x';
data = fscanf(fid,formatSpec);
idx = data > 32767;
data(idx) = data(idx) - 65536;
%validData = data(186195:end);
validData = data(1:end);
normalizedData = validData/max(validData);
%signedData = cast(data, 'int16');
plot(normalizedData)
fclose(fid);

%% 

filename = "Packet Log.txt";
packetFID = fopen(filename);
formatSpec = '%d %s %s\n';
packets = textscan(packetFID,formatSpec);
packetCount = cell2mat(packets(1,1));
packetsContinuity = ([packetCount; packetCount(end)+1] == [packetCount(1); packetCount+1]);
fclose(packetFID);

packetLoc = find(~packetsContinuity);
packetsLost = 0;
packetGaps = [];
for i = 1:length(packetLoc)
    packetGap = packetCount(packetLoc(i))-packetCount(packetLoc(i)-1);
    if packetGap < 0
        packetGap = packetGap + 255;
    end
    if packetGap > 0
        packetGaps(end+1,1) = packetGap;
    end
    packetsLost = packetsLost + packetGap;
end

packetLoc = packetLoc(packetCount(packetLoc)-packetCount(packetLoc-1) ~= -255);
