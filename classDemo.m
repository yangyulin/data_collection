clear
clc
close all
f = fopen('input.txt','r');
input = fscanf(f, 'time %f voltage %f\n',[2 inf]);
fclose(f);
%input(2,:) = input(2,:)-mean(input(2,:));

f = fopen('output.txt','r');
output = fscanf(f, 'time %f pos %f\n',[2 inf]);
fclose(f);

Fs = 500;
T = 1/Fs;

%Some data collected in the first few seconds may be a result of serial flush and should be dicarded
xstart = 500;
for i = xstart-100:xstart+100
    if abs(output(1,i)-input(1,xstart)) < 0.5*T
        ystart = i;
        break
    end
end

sync = -size(input,2) + size(output,2) - 100 + xstart - ystart - 1;
x = input(2,xstart:end+sync)';
y = output(2,ystart:end-100)'-output(2,ystart);

vel = diff(y,1)*Fs;
%%
%Create data object for Matlab System Identificatio Toolbox 
z = iddata(vel,x,T);
z.InputName = 'Voltage'; 
z.OutputName = {'AngleVel'};
z.int = 'foh';
figure()
plot(z(:,1,:));
set(gcf,'numbertitle','off','name','Raw signal') 
%%
%Move your waveform's center line to zero
%This accelerates system identification without affecting its accuracy
z = detrend(z,0);
figure()
plot(z(:,1,:));
set(gcf,'numbertitle','off','name','Raw signal moved to center') 
%%
%Use 'fft' to get z's fourier transform z_fourier
%Plot z_fourier in frequency domain and determine your low pass filter's passband

f_z = fft(z);
figure()
plot(f_z);
set(gcf,'numbertitle','off','name','Raw signal frequency response') 

%%
%Use 'idfilt' to filter z and obtain cleaner velocity data fz
%Plot fz in both time domain and frequency domain

fz = idfilt(z, [0,150]);
figure()
plot(fz);
set(gcf,'numbertitle','off','name','Filtered raw signal in time domain') 

ffz = fft(fz);

figure()
plot(ffz);
set(gcf,'numbertitle','off','name','Filtered raw signal in frequency domain')


%%
%Use 'pem' to identify motor system based on filtered data fz
%Choose 'P2U' as 'pem' parameter, which corresponds to second order
%underdamped system.
%Use 'help pem' for further information
m2u = pem(fz, 'P2U')

%%
%Plot the fitness of your model by comparing your model with filtered data
%fz using 'compare' command
figure()
compare(fz, m2u)
set(gcf,'numbertitle','off','name','M2u fit') 

%%
%Check the error of your model using 'pe' command 
figure()
pe(fz, m2u)
set(gcf,'numbertitle','off','name','M2u error') 


%%
%Plot poles of your model
%cmd 'pzmap'
figure()
pzmap(m2u)
set(gcf, 'numbertitle', 'off', 'name', 'Compare poles and zeros')

%%
%Formulate your model as transfer functions
sys2 = tf(-m2u.Kp.value, [m2u.Tw.value^2, 2*m2u.Tw.value*m2u.Zeta.value, 1])

poles = pole(sys2)

%%
%Reduce your 2nd model to 1st order
sys1 = tf(-m2u.Kp.value, [1/(-poles(2)),1])

%%
%Compare the step response of your 1st order system and 2nd order system

yt = linspace(0,0.3,100);
s = tf('s');

step2 = step(sys2,yt);
step1 = step(sys1,yt);

stepinfo(step1, yt);

figure()
plot(yt, step1, yt, step2)
legend('Reduce model', '2nd order system')
title('Comparison between reduced order system and 2nd order system')

%%
%Design a P controller to control your motor's velocity
%Check the steady state error of your P controller
%Check the voltage input produced by your P controller, be careful not to
%exceed motor shield's maximum output voltage (5V)

k = -m2u.Kp.value;
tau = -1/poles(2);
figure()
rlocus(sys1);

speed = 1000;
ts = [0.1 0.05 0.02 0.01];
kc = (4*tau./ts - 1)/k;
yt = linspace(0,0.4,1000);

sys1_cl = cell(size(ts,2));
sys1_cl_step = zeros(size(ts,2), size(yt,2));
yts = zeros(size(ts));

wc = 2*pi;
%lp = 1/(s/wc+1);
lp = 1;
for i = 1:size(ts,2)
    sys_cl{i} = feedback(sys1*kc(i),lp);
    sys1_cl_step(i,:) = speed*step(sys_cl{i}, yt);
    y = stepinfo(sys1_cl_step(i,:),yt);
    yts(i) = y.SettlingTime;
end

ref1 = linspace(0,speed,100);
ref2 = ones(size(yt))*speed;

figure()
for i = 1:size(ts,2)
    plot(yt, sys1_cl_step(i,:),'LineWidth',2,'Color', [0,i/size(ts,2),i/size(ts,2)]);
    hold on
    plot(yts(i)*ones(1,size(ref1,2)),ref1,'--','LineWidth',2,'Color', [0,i/size(ts,2),i/size(ts,2)]);
    hold on
end
plot(yt, ref2,'--g');
%axis([0 0.3 0 1.2*speed])
legend('ts=0.1','ts=0.05','ts=0.02','ts=0.001')

%%
%Plot control input required by P controller
Gu = cell(size(ts,2));
u = zeros(size(sys1_cl_step));
for i = 1:size(ts,2)
    Gu{i} = feedback(kc(i), sys1*lp);
    u(i,:) = speed*step(Gu{i}, yt);
end

figure()
for i = 1:size(ts,2)
    plot(yt, u(i,:),'LineWidth',2,'Color', [0,i/size(ts,2),i/size(ts,2)]);
    hold on
end
%axis([0 0.1 -0.1 9])
legend('ts=0.1','ts=0.05','ts=0.02','ts=0.001')

%%
%Design a PI controller to control your motor's velocity
%Check the voltage input produced by your P controller, be careful not to
%exceed motor shield's maximum output voltage (5V)

kp = kc(3);
tspi = 8*tau/(k*kp+1);
yt = linspace(0,0.15,1000);
ref2 = ones(size(yt))*1;

ratio = [0.5 1 2 5];

GcPI = cell(size(ratio,2));
yr = zeros(size(ratio,2),size(yt,2));
for i = 1:size(ratio,2)
    ki = ratio(i)*(k*kp^2+kp)/(2*tau);
    GcPI{i} = kp + ki/s;
    Gr = feedback(GcPI{i}*sys1, 1);
    yr(i,:) = speed*step(Gr,yt);
end

figure()
for i = 1:size(ratio,2)
    plot(yt, yr(i,:),'LineWidth',2,'Color', [0,i/size(ratio,2),i/size(ratio,2)]);
    hold on
end
plot(yt, ref2,'--g');
legend('ratio = 0.5', 'ratio = 1', 'ratio = 2', 'ratio = 5')

%%
%Plot control input required by PI controller
Gu = cell(size(ratio,2));
u = zeros(size(yr));
for i = 1:size(ratio,2)
    Gu{i} = feedback(GcPI{i}, sys1);
    u(i,:) = speed*step(Gu{i}, yt);
end

figure()
for i = 1:size(ratio,2)
    plot(yt, u(i,:),'LineWidth',2,'Color', [0,i/size(ratio,2),i/size(ratio,2)]);
    hold on
end
legend('ratio = 0.5', 'ratio = 1', 'ratio = 2', 'ratio = 5')
