%fclose(instrfind);
% s=serial(seriallist);
% s.BaudRate=115200;
clear;
 load('s.mat');
fclose(s);
fopen(s);
% for i=1:6
% fscanf(s)
% end
len=300;
fprintf('start');
tic
for i= 1:300
X(i)=str2num(fscanf(s));
Y(i)=str2num(fscanf(s));
Z(i)=str2num(fscanf(s));
gX(i)=str2num(fscanf(s));
gY(i)=str2num(fscanf(s));
gZ(i)=str2num(fscanf(s));
end
toc
fclose(s);
fprintf('stop');