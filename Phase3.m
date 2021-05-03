% Jamison McGinley SID: 105207291
% Simple Simulator to test GOA theories

clear;close all;clc;

%Define workspaces: obstacle code removed but simple c/p
workspace1 = zeros(4,2,5);
O1 = [1 1; 2 1; 2 5; 1 5];
O2 = [3 4; 4 4; 4 12; 3 12];
O3 = [3 12; 12 12; 12 13; 3 13];
O4 = [12 5; 13 5; 13 13; 12 13];
O5 = [6 5; 12 5; 12 6; 6 6];
workspace1(:,:,1) = O1;
workspace1(:,:,2) = O2;
workspace1(:,:,3) = O3;
workspace1(:,:,4) = O4;
workspace1(:,:,5) = O5;

%Produce one trajectory to validate confidence
%------------------------------
start = [normrnd(15,.5,1), normrnd(0,.5,1)];
goal = [normrnd(10,.5,1), normrnd(10,.5,1)];
enemy = [normrnd(5,.5,1), normrnd(2,.5,1)];

figure()
[current, adversary, caught, pzs, oa, pf_oa] = goa_online_pf3(workspace1,start,goal,enemy,0.4,500,500,inf);
subplot(2,1,1)
hold on;
grid on;
xlabel('k');
ylabel('GOA');
title('GOA value comparison');

axis equal;
title('Bug Problem Playout');
xlabel('X');
ylabel('Y');
scatter(adversary(1:5:end,1),adversary(1:5:end,2),'k');
scatter(current(1:5:end,1),current(1:5:end,2),'k');
plot(adversary(1,1),adversary(1,2),'r.','MarkerSize',10)
plot(goal(1),goal(2),'g.','MarkerSize',10)
a = plot(current(:,1),current(:,2),'b','LineWidth',5);
a.Color(4) = 0.3;
b = plot(adversary(:,1),adversary(:,2),'r','LineWidth',5);
b.Color(4) = 0.3;

subplot(2,1,2)
hold on;
grid on;
place = 1:5:length(current);
for i = 1:length(place)
    xline(place(i),'--');
end
a(1) = plot(1:length(oa),oa);
a(2) = plot(1:length(pf_oa),pf_oa);
legend(a,{'Original','New'});
xlabel('k');
ylabel('GOA');

title('GOA value comparison');
