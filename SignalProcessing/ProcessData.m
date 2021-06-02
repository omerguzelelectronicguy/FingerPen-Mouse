clc;    clear;
load('tri3.mat')
%% Moving Average Filter Implementation
%getData;
N=1;
for i=1:9
    temp=A(:,i)';
    B(:,i)=movingAverageFilter(temp,N);
end
%% Delete offset acceleration
% x=x(2:end)-x(1:end-1);x(end+1)=x(end);
% y=y(2:end)-y(1:end-1);y(end+1)=y(end);
% z=z(2:end)-z(1:end-1);z(end+1)=z(end);

% x=x'-mean(x);    y=y'-mean(y);    z=z'-mean(z);
%
% x=-filter([-0.25 -0.5 -0.75 -1 1 0.75 0.5 0.25],1, x);  x=x';
% y=-filter([-0.25 -0.5 -0.75 -1 1 0.75 0.5 0.25],1, y);  y=y';
% z=-filter([-0.25 -0.5 -0.75 -1 1 0.75 0.5 0.25],1, z);  z=z';
%%
figure;
tres=15;
for k=1:3
    x=B(:,3*k-2);
    y=B(:,3*k-1);
    z=B(:,3*k);
    
    x(abs(x)<tres)=0;
    y(abs(y)<tres)=0;
    z(abs(z)<tres)=0;
    
    len=length(x);
    dx=zeros(len,1);
    dy=zeros(len,1);
    dz=zeros(len,1);
    vx=0;vy=0;vz=0;
    for i=2:len
        if(abs(x(i))>tres || abs(y(i))>tres)
            vx=vx*1+x(i);
            vy=vy*1+y(i);
        else
            vx=0;
            vy=0;
        end
        dx(i)=dx(i-1)+vx/100;
        dy(i)=dy(i-1)+vy/100;
    end
    
    subplot(1,3,k);
    curve=animatedline('Color','r','Marker','.');
    axis([min(dx) max(dx) min(dy) max(dy)]);
    grid on; axis equal;
    for i=1:length(dx)
        addpoints(curve,dx(i),dy(i));
        drawnow;        %pause(0.01);
    end
end

