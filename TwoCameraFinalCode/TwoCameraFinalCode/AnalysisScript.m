clear times;
clear temp1electronics;
clear temp2food;
clear weight1;
clear weight2;
clear weight3;
clear battery;
fid = fopen("myData.txt", "r");
line = fgets(fid);
times = [];
temp1electronics = [];
temp2food = [];
weight1 = [];
weight2 = [];
weight3 = [];
battery = [];
lineErrors = 0;
while line ~= -1
 %disp(line);
 try
 js = jsondecode(line);
 nt = datetime(js.received_at, "InputFormat", 'yyyy-MM-dd''T''HH:mm:ss.SSSSSSSSSZ', "TimeZone", 'America/New_York');
 ntemp1electronics = js.uplink_message.decoded_payload.temp1electronics;
 ntemp2food = js.uplink_message.decoded_payload.temp2food;
 nweight1 = js.uplink_message.decoded_payload.weight1;
 nweight2 = js.uplink_message.decoded_payload.weight2;
 nweight3 = js.uplink_message.decoded_payload.weight3;
 nbattery = js.uplink_message.decoded_payload.battery;
 %change 'temp' to what you call your decoded value, like 'RH' or 'myInteger'
 %you can build lists of whatever decoded values you have
 times = [times, nt];
 temp1electronics = [temp1electronics, ntemp1electronics];
 temp2food = [temp2food, ntemp2food];
 weight1 = [weight1, nweight1];
 weight2 = [weight2, nweight2];
 weight3 = [weight3, nweight3];
 battery = [battery, nbattery];
 % the above two lines perform list concatenation.
 % we append a new value to existing lists
 catch ME

 %fprintf("Error with line:\n%s\n", line);
 lineErrors = lineErrors + 1;
 end

 line = fgets(fid);
end
%plot(times, temps, '-ok', 'MarkerFaceColor','k');
plot(times, temp1electronics)
hold on
plot(times, temp2food)
hold on
yline(85,'-','Electronics Danger Zone');
hold on
yline(60,'-','Fresh Produce Spoil Zone End');
hold on
yline(4,'-','Fresh Produce Spoil Zone Start');
hold off
% tstart = times(1)
% tend = times(21)
% xlim([tstart tend])
title('Collected Food Cabinet Data')
xlabel('Time (timestamps)')
ylabel('Temperature (C)')
legend('Electronics Temp (C)','Food Temp (C)')
figure
plot(times, weight1)
hold on
plot(times, weight2)
hold on
plot(times, weight3)
hold on
yline(0,'-','Empty');
hold on

yline(1,'-','Almost empty');
hold on
yline(3,'-','Shelf has items');
hold off
%ylim([-5 4])
title('Collected Food Cabinet Data')
xlabel('Time (timestamps)')
ylabel('Weight (kg)')
legend('Bottom Shelf Weight (kg)', 'Middle Shelf Weight (kg)', 'Top Shelf Weight (kg)')
figure
plot(times, abs(battery))
ylim([0 120])
title('Food Cabinet Battery Level')
xlabel('Time (timestamps)')
ylabel('Battery Percentage')
legend('Current Battery Percentage')