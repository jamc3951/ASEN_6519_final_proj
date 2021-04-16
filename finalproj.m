
nchar=100;
npts=100;
for i=1:nchar
    startpts(i,:) = [normrnd(10+5*rand(1),1+rand(1)), normrnd(0,1+rand(1))];    
end

figure
scatter(startpts(:,1),startpts(:,2))

select=randi([1 npts],[1,nchar])