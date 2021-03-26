function [outcomes,bins,xBin] = composite_ordering(traj,d_0)
e = 1; %dist to goal
b1 = [];
b2 = [];
b3 = [];
b4 = [];
xBin = zeros(1,length(traj));

traj = traj(:,2:3); % [d2g, goal/no goal]

for i = 1:size(traj,1)
    if traj(i,2) == 1 && traj(i,1) < e %Bin 1
        b1 = [b1, i];
        xBin(i) = 1;
    end
    if traj(i,2) == 0 && traj(i,1) < e %Bin 2
        b2 = [b2, i];
        xBin(i) = 2;
    end  
    if traj(i,2) == 0 && traj(i,1) > e && traj(i,1) < d_0 %Bin 3
        b3 = [b3, i];
        xBin(i) = 3;
    end      
    if traj(i,2) == 0 && traj(i,1) > d_0  %Bin 4
        b4 = [b4, i];
        xBin(i) = 4;
    end         
end

outcomes = [b4,b3,b2,b1];
bins = [length(b4),length(b3),length(b2),length(b1)];
end

