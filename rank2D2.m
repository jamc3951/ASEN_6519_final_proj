function [scaled] = rank2D2(traj)
% Change # of steps survived to D2Goal

worst_d2g = max(traj(:,2));

caught = (find(traj(:,3) == 0));

for i = 1:length(caught)
    caught(i) = 0 + (abs((traj(i,3)-worst_d2g)/worst_d2g));
    %Formula x1 [0,1] + relative success
end
caught = sort(caught);

notcaught = (find(traj(:,2) == 1));

for i = 1:length(notcaught)
    notcaught(i) = 1 + (abs((traj(i,3)-worst_d2g)/worst_d2g));
end
%Sort for GOA
notcaught = sort(notcaught);

%Scale for binning process
scaled = vertcat(caught,notcaught);
end
