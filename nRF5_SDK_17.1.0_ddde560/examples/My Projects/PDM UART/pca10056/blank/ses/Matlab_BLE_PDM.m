clear
clc

filename = "PDM BLE Test 3-28-22.txt";
fid = fopen(filename,'w');
fclose(fid);

filename = "Packet Log.txt";
fid = fopen(filename,'w');
fclose(fid);

nordic = ble("C44A3A76D854");
datachar = characteristic(nordic, "6E400001-B5A3-F393-E0A9-E50E24DCCA9E", "6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
datachar.DataAvailableFcn = @displayCharacteristicData;

function displayCharacteristicData(src,~)
    fid = fopen("PDM BLE Test 3-28-22.txt",'a+');
    [data,timestamp] = read(src,'oldest');

    %sound(data(2:end),8000);

    for i = 2:2:length(data)
        fprintf(fid,'%x%x ', data(i),data(i+1));
    end
    fprintf(fid, '\n');
    fclose(fid);
    packetLog = fopen("Packet Log.txt",'a+');
    fprintf(packetLog,'%d %s\n',data(1),timestamp(1));
    fclose(packetLog);
end