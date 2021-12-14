%P190




clear
clc
format bank
%p190
%data format
%year doy refHt azimuth fr hrs sat ? voltssnr pk2n
%%
fidfire = fopen('p190fire.txt');
firesnr = textscan(fidfire, '%f %f %f %f %f %f %f %f %f %f %f %f', -1,'headerlines',1);
%%
fidcont = fopen('p190cont.txt');
contsnr = textscan(fidcont, '%f %f %f %f %f %f %f %f %f %f %f %f', -1,'headerlines',1);
%%
%Fire array format
%Messing around with snr in order to find good azimuth in line 18
firearray1 = cell2mat(firesnr(1:end,1:9));
firearray = sortrows(firearray1,[2 6]);
firepksnr = firearray1(1:end,9);
fireaz = firearray(1:end,4);
fireazindex = find(fireaz<180)
fireazindex= firepksnr(fireaz)
fireazindex = fireazindex(1:80)
%fireazindex = fireazindex;
%length(fireazindex)
%%
%Control array format
contarray1 = cell2mat(contsnr(1:end,1:9));
contarray = sortrows(contarray1,[2 6]);
contpksnr = contarray(1:end,9);
contaz = contarray(1:end,4);
contazindex = find(contaz<180);
contazindex = contpksnr(contazindex);
%contazindex = contazindex(1:380);
%length = 105 for az index
%%
%clc
%Azimuth vs Snr
%azimuth = firearray(1:end,4)
%SNR = firearray(1:end,9)
%nr = [azimuth SNR]

%polarscatter(azimuth,SNR,'filled')
%hold on 
%azimuth2 = contarray(1:end,4)
%SNR2 = contarray(1:end,9)
%polarscatter(azimuth2,SNR2,'filled')

%scatter(SNR,azimuth,'filled','color','r')
%hold on
%scatter(SNR2,azimuth2,'filled','color','r')

%%
%Normal plots

csmooth= smooth(contazindex);
fsmooth = smooth(fireazindex);
figure
plot(fsmooth,'color','r','linewidth',1.5)
hold on
%yline(mean(fsmooth),'color','r');
plot(csmooth,'color','b','linewidth',1.5)
%yline(mean(csmooth),'color','b');
title('Moving Average, P190 Azimuth Specific (degrees>250)')
ylabel('Peak to SNR (v/v)')
xlabel('time')
xlim([1 80]);
%%
logf = log(fireazindex)
logc = log(contazindex)
figure
plot(logf)
hold on
plot(logc)
%%
%testing out stats stuff
[h,p,ci,stats] = ttest2(fireazindex,contazindex,"Alpha",0.95)

figure
pd = fitdist(fireazindex,'Normal');
x_values = min(fireazindex):.1:max(fireazindex);
y = pdf(pd,x_values);
hold on;
plot(x_values,y,'LineWidth',2)
meanfire = mean(fireazindex);
stdfire = std(fireazindex);
%xline([meanfire-stdfire meanfire meanfire+stdfire],'-',{'-1 Std Dev.','Average','+1 Std Dev.'})

pd1 = fitdist(contazindex,'Normal');
x_values1 = min(contazindex):.1:max(contazindex);
y1 = pdf(pd1,x_values1);
plot(x_values1,y1,'LineWidth',2);
meancont = mean(contazindex);
stdcont = std(contazindex);
%xline([meancont-stdcont meancont meancont+stdcont],'-',{'-1 St. Dev.','Average','+1 Std. Dev.'})
%%
%log10(fireazindex)







%%
%Distribution Plots
figure
histogram(fireazindex,10)
title('fire histogram')
xlabel('Mean = 5.30')
figure
histogram(contazindex,10)
title('control histogram')
xlabel('Mean = 4.05')
mean(contazindex)
mean(fireazindex)