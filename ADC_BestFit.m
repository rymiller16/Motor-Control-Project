%Best fit line for ADC converter 

%Trial 1 - Not Averaged
% x = [566, 560, 521, 514, 512, 504, 508, 472, 432];        %adc counts
% y = [192, 52.3, 25.9, 17.5, 0, -17.5, -25.8, -53, -195];    %measured current
% 
% plot(x,y);
% p = polyfit(x,y,1)
% xlabel('ADC Counts');
% ylabel('Measured Current (mA)');

%Trial 2 - Averaged
% x = [566, 524, 504, 496, 492, 464, 433]
% y = [198.6, 54.9, 26, 0, -26, -55.2, -185.3]
% 
% plot(x,y);
% p = polyfit(x,y,1)
% xlabel('ADC Counts');
% ylabel('Measured Current (mA)');

%Trial 3 - Averaged with higher cutoff Frequency
x = [562, 517, 505, 500, 493, 490, 433]
y = [190.1, 53.9, 25.1, 0, -25.2, -53.8, -189.4]

plot(x,y);
p = polyfit(x,y,1)
xlabel('ADC Counts');
ylabel('Measured Current (mA)');