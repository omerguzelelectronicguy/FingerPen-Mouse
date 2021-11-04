clear;
clc;
load("data7Vel.mat");
t_vy = t(:,3);
t_vx = t(:,4);
t_ay = t(:,5);
t_ax = t(:,6);
s_vx = s(:,1);
s_vy = s(:,2);


ts = linspace(0.8,5,501);
tt = linspace(0,5,721);
subplot 211
plot(tt,t_vy)
hold on;

plot(ts,s_vy);
legend("tracker","sensor");
hold off;

subplot 212
hold on;

plot(tt,t_vx)
plot(ts,-s_vx);
legend("tracker","sensor");
hold off;


t_vx_cal = 0*t_ax;
t_vy_cal = 0*t_ay;

for i=3:length(t_ax)
    t_vx_cal(i)=sum(t_ax(3:i))/25;
    t_vy_cal(i)=sum(t_ay(3:i))/25;
end

figure;
subplot 211
plot(tt,t_vy_cal)
hold on;

plot(ts,s_vy);
legend("tracker","sensor");
hold off;

subplot 212
hold on;

plot(tt,t_vx_cal)
plot(ts,-s_vx);
legend("tracker","sensor");
hold off;

%%%%%%%%%%%%%%%%%%%%%%%555
figure;
subplot 211
plot(tt,t_vy_cal)
hold on;

plot(tt,t_vy);
legend("tracker_cal","tracker");
hold off;

subplot 212
hold on;

plot(tt,t_vx_cal)
plot(tt,t_vx);
legend("tracker_cal","tracker");
hold off;