fclose('all');  close all;     clear;  clc;
temp = input('Participiant Number: ');
fileName = num2str(temp);
fileName = ['p',fileName,'_Circle.txt'];
fileID = fopen(fileName,'w');
fprintf(fileID,'%s\n',string(datetime('now')));
fprintf(fileID,'x:\ty:\tnewError:\ttotalError:\ttime:\n');
t=0:0.001:2*pi;
setGlobal(cos(t),sin(t),fileID);
[tx,ty] =getGlobal;

plot(tx, ty);   hold on;
plot(tx(1), ty(1),'o','color','c');
xlim([-1.2 1.2]);   ylim([-1.2 1.2]);
set (gcf, 'WindowButtonMotionFcn', @mouseMove);

function mouseMove (object, eventdata)
    C = get (gca, 'CurrentPoint');
    global fileID
    [tx,ty] =getGlobal;
    x = C(1,1);
    y = C(1,2);
    fprintf(fileID,'%f\t%f',x, y);
    if x>-1.2 && x < 1.2 && y>-1.2 && y<1.2
        dx = (tx-x).^2;
        dy = (ty-y).^2;
        d = min(dx+dy)*100;
        changeError(d);
        title(gca, ['(X,Y) = (', num2str(C(1,1)), ', ',num2str(C(1,2)), ') - Error',num2str(d),'                     ']);
    else
        disp('BE IN THE GRAPH AREA!!!');
        title(gca, ['(X,Y) = (', num2str(C(1,1)), ', ',num2str(C(1,2)), ') - BE IN THE GRAPH AREA!!!','        ']);
        fprintf(fileID,'\n');
    end
end

function setGlobal(val1,val2,fid)
    global fileID
    global tx
    global ty
    global errorsum
    global errorRes
    global sampleNumber
    global start_flag
    fileID = fid;
    tx = val1;
    ty = val2;
    errorsum =0;
    sampleNumber=0;
    errorRes = 0;
    start_flag = 1;
end

function changeError(newError)
    global errorsum
    global errorRes
    global sampleNumber
    global fileID
    global start_flag
    if start_flag == 1
            tic;
            start_flag = 0;
    end
    errorsum = errorsum + newError;
    sampleNumber = sampleNumber+1;
    errorRes = errorsum / sampleNumber;
    t_elapsed=toc;
    fprintf(fileID,'\t%f\t%f\t%f\n', newError, errorRes, t_elapsed);
end

function [a,b] = getGlobal
    global tx
    global ty
    a=tx;
    b=ty;
end