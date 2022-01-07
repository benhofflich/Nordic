clear
close
hrLog = [];
spo2Log = [];
disp("Connecting to MAX32664 Sensor Hub")
device = ble("Nordic_HRM");
disp("Reading Heart Rate")
hr_char = characteristic(device,"180D", "2A37");
disp("Reading SpO2")
spo2_char = characteristic(device,"1822", "2A5F");
subscribe(hr_char)
subscribe(spo2_char)
plot(hrLog,'b');
hold on
plot(spo2Log,'r');
i = 1;
while(i<=400)
    try
        heartrateTemp = read(hr_char);
        heartrate = heartrateTemp(2);
        oxyTemp = read(spo2_char);
        oxy = oxyTemp(2);
    catch
    end
    hrLog(i) = heartrate;
    spo2Log(i) = oxy;
    plot(hrLog,'b')
    plot(spo2Log,'r')
    %pause(0.05)
    i = i + 1;
end
%%
time = 0.05:0.05:20;
yyaxis left
plot(time,hrLog)
ylabel("Heartrate (BPM)")
ylim([50,100])
yyaxis right
plot(time,spo2Log)
ylabel("SpO2 (%)")
ylim([80, 100])
title("Heart Rate and SpO2")
xlabel("Time (s)")