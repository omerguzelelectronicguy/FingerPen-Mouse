function out=movingAverageFilter(in,N)
out=zeros(size(in,2)-N,1);
for i=1:(size(in,2)-N)
    out(i)=sum(in(i:i+N-1))/N;
end

end