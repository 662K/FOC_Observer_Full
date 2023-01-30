clc;clear;

load("SpdE.mat")

t = SpdE(:,1);
Spd = SpdE(:,2);
Target_Spd = SpdE(:,3);

plot(t,Spd); hold on; grid on;
plot(t,Target_Spd); hold off;

xlim([0.05 0.35])

xlabel("\textbf{Time(s)}");
ylabel("\textbf{Actual and Targeted Spd(RPM)}");
title("");
legend('Actual spd', 'Targeted spd','Location' ,'southeast');