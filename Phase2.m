clearvars -except simcurr simadv simoutc
close all
clc
% 
% tic
% load('sim_adv')
% load('sim_curr')
% load('sim_outc')
% toc
%% All the fancy algorithms

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


start = [normrnd(15,1,1), normrnd(0,1,1)];
goal = [normrnd(10,1,1), normrnd(10,1,1)];
enemy = [normrnd(5,6,1), normrnd(2,6,1)];

% [current, adversary, goal_achieved] = bug1(workspace1,start,goal,enemy,.4,inf);

[current, adversary, caught, pzs, oa]=goa_online_no_composite(workspace1,start,goal,enemy,.4,1000,inf);

goal_achieved = ~caught(end)
successbin=zeros(1,length(current));
failurebin=zeros(1,length(current));
closeness = 0.05;


%%
w_consid=length(simcurr); % Start with considering whole dataset
weight=ones(w_consid,1)./(w_consid);
for k=1:length(current)
    tic
    for j=1:w_consid
        if length(simcurr{j})>= k
            weight(j)=mvnpdf([(simcurr{j}(:,k))', (simadv{j}(:,k))'], [current(k,:), adversary(k,:)], 10*eye(4));
        else
            weight(j)=0;
        end
    end
    toc
    
    weight=weight./sum(weight);
    
    
    
    [sort_weight,idx_weight]=sort(weight,'descend');
    keepidx=find(sort_weight>=max(sort_weight)*closeness);
    
    weight=sort_weight;
    w_consid=length(keepidx);
    simcurr=simcurr(idx_weight);
    simadv=simadv(idx_weight);
    simoutc=simoutc(idx_weight);
    successbin(k)=simoutc*weight;
    failurebin(k)=1-successbin(k);
    
    [goal_confidence,z,z_ll,p_z] = general_oa_v2(datasample([0,1],500,'Weights',[failurebin(k) successbin(k)]), [-0.5,0.5,1], 2);
    goaest(k)= goal_confidence;
    
end


figure
plot(successbin)
xlabel('Time step (k)')
ylabel('Estimated Probability of Success')
ylim([0 1])
title('Particle Filter Probability Estimate', 'For the Successful Case')

figure
plot(goaest)
hold on
plot(oa)



figure, plot(current(:,1), current(:,2))
hold on
plot(adversary(:,1), adversary(:,2))

