clear; clc;

Data = readtable("DataRec.csv");

Real_ThetaE = Data.I7;
Observer_ThetaE = Data.I6;

Real_ThetaE = Real_ThetaE(30000 : 49000);
Observer_ThetaE = Observer_ThetaE(30000 :49000);

f1 = figure(1);
plot(Real_ThetaE); hold on;
plot(Observer_ThetaE); hold off;

% Real_ThetaE1 = Real_ThetaE(3750 : 6789);
% Observer_ThetaE1 = Observer_ThetaE(3750 : 6789);

a1 = 0.1199;
b1 = 1.933;
c1 = 0.2675;
d1 = 0.1446;

Observer_ThetaE = Observer_ThetaE + a1*sin(b1*Observer_ThetaE + c1) + d1;

Observer_ThetaE = rem(Observer_ThetaE, 2*pi());

f2 = figure(2);
plot(Real_ThetaE); hold on;
plot(Observer_ThetaE); hold off;

Observer_Error = Real_ThetaE - Observer_ThetaE;

for i = 1:length(Observer_Error)
    if(Observer_Error(i) < -pi())
        Observer_Error(i) = Observer_Error(i) + 2*pi(); 
    end
end

f3 = figure(3);
plot(Observer_Error);
