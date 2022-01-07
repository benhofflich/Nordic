clear
hrLog = [];
disp("Connecting to MAX32664 Sensor Hub")
device = ble("Nordic_HRM");
disp("Reading Heart Rate")
hr_char = characteristic(device,"180D", "2A37");
subscribe(hr_char)
plot(hrLog);
i = 1;
while(i<=400)
    try
        heartrateTemp = read(hr_char);
        heartrate = heartrateTemp(2);
    catch
    end
    hrLog(i) = heartrate;
    plot(hrLog)
    pause(0.05)
    i = i + 1;
end
%%
time = 0.05:0.05:20;
plot(time,hrLog)
title("Heart Rate")
ylabel("Heartrate (BPM)")
xlabel("Time (s)")