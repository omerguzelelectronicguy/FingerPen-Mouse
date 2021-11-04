clc;    clear;  close all;
load('data1.mat');

plot(sensor_vy);
title("sensor data");

tracker_vx = trackveri(:,6);
tracker_vy = trackveri(:,5);
figure;
plot(tracker_vy);
title("tracker data");
tr_x =tracker_vx;
tr_y =tracker_vy;
for i=2:213
    tr_x(i) = sum(tracker_vx(1:i));
    tr_y(i) = sum(tracker_vy(1:i));
end
figure;
plot(tr_x);
title("tracker data");

s_x =sensor_vx;
s_y =sensor_vy;
for i=2:310
    s_x(i) = sum(sensor_vx(1:i));
    s_y(i) = sum(sensor_vy(1:i));
end
figure;
plot(s_x);