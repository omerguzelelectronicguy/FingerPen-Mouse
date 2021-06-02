clear;
curve=animatedline('Color','r','Marker','.');
% axis([min(yy_filtered) max(yy_filtered)...
%       min(xx_filtered) max(xx_filtered)]);
grid on;

 load('s.mat');
fclose(s);
fopen(s);
vx=0;vy=0;vz=0;dx=0;dy=0;dz=0;
while 1
len=8;
for i= 1:len
x(i)=str2num(fscanf(s));
y(i)=str2num(fscanf(s));
z(i)=str2num(fscanf(s));
end
N=6;
x=movingAverageFilter(x,N);
y=movingAverageFilter(y,N);
z=movingAverageFilter(z,N);

%% Delete offset acceleration
x=x(2:end)-x(1:end-1);
y=y(2:end)-y(1:end-1);
z=z(2:end)-z(1:end-1);
% 
% x=-filter([-0.25 -0.5 -0.75 -1 1 0.75 0.5 0.25],1, x);
% y=-filter([-0.25 -0.5 -0.75 -1 1 0.75 0.5 0.25],1, y);
% z=-filter([-0.25 -0.5 -0.75 -1 1 0.75 0.5 0.25],1, z);
%%
len=300;
vx=vx+x.*10;        vy=vy+y.*10;        vz=vz+z*10;
dx=dx+vx+x*10;
dy=dy+vy+y*10;
dz=dz+vz+z*10;
addpoints(curve,dy,dx);
drawnow;
end