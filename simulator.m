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

start = [normrnd(15,.5,1), normrnd(0,.5,1)];
goal = [normrnd(10,.5,1), normrnd(10,.5,1)];
enemy = [normrnd(5,.5,1), normrnd(2,.5,1)];

%Run Bug algorithms and plot
%Every time agent moves, the enemy moves towards agent

%Option 2: Recompute MC's in discrete time
figure()
[current, adversary, caught, pzs, oa, pf_oa] = goa_online_plusPF(workspace1,start,goal,enemy,0.4,500,500,inf);
subplot(2,1,1)
hold on;
grid on;

plot(1:length(oa),oa);
plot(1:length(pf_oa),pf_oa);
legend('Original','New');
xlabel('k');
ylabel('GOA');
title('GOA value comparison');

N_s = 500;
Nsim=50;
C_s = [8,20,100,500];
all_vars = zeros(Nsim,4);
pferror=zeros(60,4);
counts=zeros(60,1);
mssd=zeros(60,4);
Nsim=10;

%f = waitbar(0,'Running Simulations');
for k=1:Nsim
    [current, adversary, caught, pzs, oa, pf_oa] = goa_online_plusPF(workspace1,start,goal,enemy,0.4,N_s,C_s,inf);
    all_vars(k,:) = [var((pf_oa(:,1) - oa)) var((pf_oa(:,2) - oa)) var((pf_oa(:,3) - oa)) var((pf_oa(:,4) - oa))];
    pferror(1:length(oa),:)=pferror(1:length(oa),:)+abs(pf_oa(:,:) - oa);
    counts(1:length(oa))=counts(1:length(oa))+1;
    %waitbar(k/Nsim,f)
end

temp_cutoff = max(find(counts == Nsim));
vars_cutoff = max(find(all_vars(:,1) >0));

figure()
hold on;
grid on;
plot(pferror(1:temp_cutoff,:)./counts(1:temp_cutoff))

legend('Cs = 8','Cs = 20','Cs = 100','Cs = 500');

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
C_s = [1:50:500];
Nl = C_s;
%[BS,R,R_pf] = brier_score_on(Nl,C_s,0.2,workspace1,2,inf);


N_s = 500;
Nsim = 100;
C_s = [8,20,100,500];
all_vars = zeros(Nsim,4);
pferror=zeros(60,4);
counts=zeros(60,1);
mssd=zeros(60,4);

prev_error=pferror;
%f = waitbar(0,'Running Simulations');
for k=1:Nsim
    start = [normrnd(15,.5,1), normrnd(0,.5,1)];
    goal = [normrnd(10,.5,1), normrnd(10,.5,1)];
    enemy = [normrnd(5,.5,1), normrnd(2,.5,1)];
    [current, adversary, caught, pzs, oa, pf_oa] = goa_online_plusPF(workspace1,start,goal,enemy,0.4,N_s,C_s,inf);
    all_vars(k,:) = [var(abs(pf_oa(:,1) - oa)) var(abs(pf_oa(:,2) - oa)) var(abs(pf_oa(:,3) - oa)) var(abs(pf_oa(:,4) - oa))];
    pferror(1:length(oa),:)=pferror(1:length(oa),:)+abs(pf_oa(:,:) - oa);
    counts(1:length(oa))=counts(1:length(oa))+1;
    if k>1
        mssd(1:length(oa),:)=mssd(1:length(oa),:)+(pferror(1:length(oa),:)-prev_error(1:length(oa),:)).^2/2;
    end
    prev_error(1:length(oa),:)=abs(pf_oa(:,:) - oa);
    % for i = 1:length(C_s)
    %     plot(1:length(oa),abs(pf_oa(:,i) - oa),'LineWidth',1.1);
    % end
    %waitbar(k/Nsim,f)
end

temp_cutoff = max(find(counts == Nsim));


figure()
hold on;
grid on;
plot(pferror(1:temp_cutoff,:)./counts(1:temp_cutoff))
%errorbar(pferror(1:temp_cutoff,:)./counts(1:temp_cutoff),2*sqrt(all_vars(1:temp_cutoff,:)));
% plot(mssd./(2*(counts-1)),'--')
legend('Cs = 8','Cs = 20','Cs = 100','Cs = 500');
xlabel('k');
ylabel('Mean GOA Error');
title('Mean Original vs. New Error');


figure();
subplot(2,2,1)
hold on;
grid on;
title(sprintf('Cs=%.0f',C_s(1)))
plot(pferror(1:vars_cutoff,1)./counts(1:vars_cutoff));
plot(pferror(1:vars_cutoff,1)./counts(1:vars_cutoff)+2*sqrt(all_vars(1:vars_cutoff,1)),'--r');
plot(pferror(1:vars_cutoff,1)./counts(1:vars_cutoff)-2*sqrt(all_vars(1:vars_cutoff,1)),'--r');
legend('Error','2\sigma Bound')

subplot(2,2,2)
hold on;
grid on;
title(sprintf('Cs=%.0f',C_s(2)))
plot(pferror(1:vars_cutoff,2)./counts(1:vars_cutoff));
plot(pferror(1:vars_cutoff,2)./counts(1:vars_cutoff)+2*sqrt(all_vars(1:vars_cutoff,2)),'--r');
plot(pferror(1:vars_cutoff,2)./counts(1:vars_cutoff)-2*sqrt(all_vars(1:vars_cutoff,2)),'--r');
legend('Error','2\sigma Bound')

subplot(2,2,3)
hold on;
grid on;
title(sprintf('Cs=%.0f',C_s(3)))
plot(pferror(1:vars_cutoff,3)./counts(1:vars_cutoff));
plot(pferror(1:vars_cutoff,3)./counts(1:vars_cutoff)+2*sqrt(all_vars(1:vars_cutoff,3)),'--r');
plot(pferror(1:vars_cutoff,3)./counts(1:vars_cutoff)-2*sqrt(all_vars(1:vars_cutoff,3)),'--r');
legend('Error','2\sigma Bound')

subplot(2,2,4)
hold on;
grid on;
title(sprintf('Cs=%.0f',C_s(4)))
plot(pferror(1:vars_cutoff,4)./counts(1:vars_cutoff));
plot(pferror(1:vars_cutoff,4)./counts(1:vars_cutoff)+2*sqrt(all_vars(1:vars_cutoff,4)),'--r');
plot(pferror(1:vars_cutoff,4)./counts(1:vars_cutoff)-2*sqrt(all_vars(1:vars_cutoff,4)),'--r');
legend('Error','2\sigma Bound')
%Option 3: Shorter horizon MC sims
% figure()
% subplot(2,1,1)
% tic
% [agent3,enemy3,caught,pz, goa_k] = goa_online(workspace1,start,goal,enemy,0.4,1000,20);
% plotWorkspace(start, goal, 'Short Horizon MCSims',agent3,enemy3,goa_k)
% toc
%
% % Baseline
% %%%%%%%%%%%
% error = 0.4;
%
%
% % Get Sims
%  traj = runArbMCSims(workspace1,1000,error,inf);
% %
% % % Compute GOA's
%  [goal_confidence,z,z_ll,p_z] = general_oa_v2(traj(:,3), [-0.5,0.5,1], 2);
% disp(['Baseline Confidence of reaching goal: ', string(goal_confidence)]);
%
% % Composite
% d_0 = distance(start,goal);
% [outcomes,bins,xBin] = composite_ordering(traj,10.66);
% [composite_confidence,~,pz] = goa_v3(outcomes,4,bins,xBin,4);
% disp(['Baseline Comp. Confidence of reaching goal: ', string(composite_confidence)]);
%
% % labels = {-1: "Very Bad", -0.5: "Bad", -0.1: "Fair", 0.1: "Good", 0.5: "Very good"}
%
%
% % Compare to Online Assessment
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% % Composite Label -- change to goal confidence due to arb. d0
% labels = [-1,-0.5,-0.1,0.1,0.5];
% lower = labels((labels < composite_confidence));
% upper = labels((labels > composite_confidence));
% base_label = [lower(1)] ;

% Accuracy
%[p] = benchmark_online(base_label,[500,250,100,50,20,10,1],0.4,workspace1,20);
%benchmark_online_error(workspace1);


%Brier Score
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Need to get offline forecast

%brier_score_off(linspace(1,500,5),0.4,workspace1,-0.5:1:1.5,2,inf);
%brier_score_on(linspace(1,300,5),0.4,workspace1,2,inf);






