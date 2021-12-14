clear
clc
format bank
%%
% TIME (GPST)   SAT  AZ(deg)  EL(deg) SNR(dBHz)   L1 MP(m)
%40-45
%%
clc
firearray = xlsread('p1902020.xlsx');
fireSNR = firearray(1:end,5);
fireAzimuth = firearray(1:end,3);
fireElevation = firearray(1:end,4);
[ElevationControl1] = find(fireAzimuth<90 & fireElevation<50 &fireElevation>5);
FireParam = firearray(ElevationControl1,1:6);
FPL = length(FireParam);
fireSNR2 = FireParam(1:end,5);
fireAzimuth2 = FireParam(1:end,3);
fireTime = FireParam(1:end,1);

contarray = xlsread('p1902019.xlsx');
contSNR = contarray(1:end,5);
contAzimuth = contarray(1:end,3);
contElevation = contarray(1:end,4);

[ElevationControl2] = find(contAzimuth<90 & contElevation<50 & contElevation>5); 
ContParam = contarray(ElevationControl2, 1:6);
contTime = ContParam(1:end,1);
[a,b] = ismember(fireTime,contTime);
a1 = ContParam(a,:);
a2 = sortrows(a1,1);
ContParam = a2;
idx = find(ismember(fireTime,contTime));
aa = FireParam(idx,:);
aa2 = sortrows(aa,1);
FireParam = aa2;

fireSNR2 = FireParam(1:end,5);
fireAzimuth2 = FireParam(1:end,3);
fireElevation = FireParam(:,4)
fireElevationSort = sortrows(FireParam,4);



contSNR2 = ContParam(1:end,5);
contAzimuth2 = ContParam(1:end,3);
contElevation = ContParam(1:end,4);
contElevationSort = sortrows(ContParam,4);

%%
%Stats

%Later variable assignnment
fnr2 = fireElevationSort(:,5);
cnr2 = contElevationSort(:,5);
firewave1 = fnr2;
contwave1 = cnr2;


%RMS
firerms = (rms(firewave1,2));
contrms = (rms(contwave1,2));
RMSdifference = rms(firewave1,2) - rms(contwave1,2);
[RMSArray] = [contElevationSort(:,4) RMSdifference];
meanrms = mean(RMSdifference);
stdrms = std(RMSdifference);
distRMS = ([meanrms-stdrms*3 meanrms-stdrms*2 meanrms-stdrms meanrms meanrms+stdrms meanrms+stdrms*2 meanrms+stdrms*3]);

[ff] = find(RMSArray(:,2) <= distRMS(1));

std2rms = RMSArray(ff,:);
mean(std2rms(:,1))
%%
%Binomial distribution of RMS Residuals
figure
pd = fitdist(RMSdifference,'Normal');
x_values = min(RMSdifference):.1:max(RMSdifference);
y = pdf(pd,x_values);
hold on;
plot(x_values,y,'LineWidth',2,'color','r')
meanfire = mean(RMSdifference);
stdfire = std(RMSdifference);
xline([distRMS(3:5)],'--',{'-1 StD.','Mean','+1 StD.'})
title('Frequency of RMS Residuals at P190')
ylabel('Frequency')
xlabel('Residual')
distRMS(1)

%%
%Altitude calculations
%P190 distance to fire perimeter 18 miles
%Mean elevation, 9 degrees
%tan(theta) = opposite/adjacent
tt = tand(9)
d = 30
altKM = tt*d
%%
fnr2 = fireElevationSort(:,5)
cnr2 = contElevationSort(:,5)
fsmooth = smooth(fnr2)
csmooth = smooth(cnr2)

figure
plot(fireElevationSort(:,4),fsmooth,'color','r')
hold on
plot(fireElevationSort(:,4),csmooth,'color','b')
title('P190 SNR vs Elevation Angle')
ylabel('SNR (dB/Hz)')
xlabel('Elevation Angle (Degrees)')
legend('2020','2019')

%%
%RMS Difference graph
figure
firewave1 = fnr2
contwave1 = cnr2
RMSdifference = rms(contwave1,2) - rms(firewave1,2)
plot(fireElevationSort(:,4),smooth(RMSdifference))
mean(RMSdifference(1:250))
%hold on
%plot(rms(firewave1,2))
%plot(rms(contwave1,2))
%(2346:2479)
%%





%%
figure
pd = fitdist(fireSNR2,'Normal');
x_values = min(fireSNR2):.1:max(fireSNR2);
y = pdf(pd,x_values);
hold on;
plot(x_values,y,'LineWidth',2,'color','r')
meanfire = mean(fireSNR2);
stdfire = std(fireSNR2);
xline([meanfire],'--','Color','r')

pd1 = fitdist(contSNR2,'Normal');
x_values1 = min(contSNR2):.1:max(contSNR2);
y1 = pdf(pd1,x_values1);
plot(x_values1,y1,'LineWidth',2,'color','b');
meancont = mean(contSNR2);
stdcont = std(contSNR2);
xline([meancont],'--','color','blue')
title('P190 SNR Distributions: 2019 vs 2020')
legend('2020','Mean 2020','2019','Mean 2019')
xlabel('SNR (dB/Hz)')
ylabel('Frequency')

meanfire
stdfire

meancont
stdcont

