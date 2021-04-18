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

start = [normrnd(15,2,1), normrnd(0,2,1)];
goal = [normrnd(10,2,1), normrnd(10,2,1)];
enemy = [normrnd(5,2,1), normrnd(2,2,1)];

%Run Bug algorithms and plot
%Every time agent moves, the enemy moves towards agent

%Option 2: Recompute MC's in discrete time
figure()
[current, adversary, caught, pzs, oa, pf_oa] = goa_online_plusPF(workspace1,start,goal,enemy,0.4,500,500,inf);
hold on;
grid on;
plot(1:length(oa),oa);
plot(1:length(pf_oa),pf_oa);
legend('Original','New');
xlabel('k');
ylabel('GOA');
title('GOA value comparison');

N_s = 500;

C_s = [5,20,100,500];
pferror=zeros(50,4);
counts=zeros(50,1);
figure()
hold on;
grid on;
for k=1:50
    [current, adversary, caught, pzs, oa, pf_oa] = goa_online_plusPF(workspace1,start,goal,enemy,0.4,N_s,C_s,inf);
    for i=1:length(C_s)
        pferror(1:length(oa),i)=pferror(1:length(oa),i)+abs(pf_oa(:,i) - oa);
    end
    counts(1:length(oa))=counts(1:length(oa))+1;
    % for i = 1:length(C_s)
    %     plot(1:length(oa),abs(pf_oa(:,i) - oa),'LineWidth',1.1);
    % end
end

plot(pferror./counts)
legend('Cs = 5','Cs = 20','Cs = 100','Cs = 500');
xlabel('k');
ylabel('Mean GOA Error');
title('Mean Original vs. New Error');

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






