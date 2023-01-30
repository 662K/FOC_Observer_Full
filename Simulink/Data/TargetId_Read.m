clc;clear;

load("TargetId.mat")

t = TargetId(:,1);
Id = TargetId(:,2);
Target_Id = TargetId(:,3);

plot(t,Id); hold on; grid on;
plot(t,Target_Id); hold off;

xlim([0.09 0.13])
ylim([-0.2 1.2])

xlabel("\textbf{Time(s)}");
ylabel("\textbf{Actual and Targeted Id(A)}");
title("");
legend('Actual Id', 'Targeted Id','Location' ,'southeast');