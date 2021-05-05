clearvars -except simcurr simadv simoutc
close all
clc
% 
% tic
 load('simadv')
 load('simcurr')
 load('simoutc')
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
Nsim = 1;
count = zeros(1,60);



for sim = 1:Nsim

start = [normrnd(15,.5,1), normrnd(0,.5,1)];
goal = [normrnd(10,.5,1), normrnd(10,.5,1)];
enemy = [normrnd(5,.5,1), normrnd(2,.5,1)];

% [current, adversary, goal_achieved] = bug1(workspace1,start,goal,enemy,.4,inf);

[current, adversary, caught, pzs, oa]=goa_online_no_composite(workspace1,start,goal,enemy,.4,1000,inf);

goal_achieved = ~caught(end);
successbin=zeros(1,length(current));
failurebin=zeros(1,length(current));
closeness = 0.1;


goaest=[];
w_consid=length(simcurr); % Start with considering whole dataset
weight=ones(w_consid,1)./(w_consid);
for k=1:length(current)-1

    for j=1:w_consid
        if length(simcurr{j})>= k
            weight(j)=mvnpdf([(simcurr{j}(:,k))', (simadv{j}(:,k))'], [current(k,:), adversary(k,:)], 1*eye(4));
            if simoutc(j)==1
                successbin(k)=successbin(k)+weight(j);
            else
                failurebin(k)=failurebin(k)+weight(j);
            end
        else
            weight(j)=0;
        end
    end

    
    weight=weight./sum(weight);
    sbt= successbin(k)+failurebin(k);
    if sbt == 0
        sim = sim-1;
        break
    end
    successbin(k) = successbin(k)/sbt;
    failurebin(k) = failurebin(k)/sbt;
    
    
    
    [sort_weight,idx_weight]=sort(weight,'descend');
    keepidx=find(sort_weight>=max(sort_weight)*closeness);
    
    weight=sort_weight;
    w_consid=length(keepidx);
    simcurr=simcurr(idx_weight);
    simadv=simadv(idx_weight);
    simoutc=simoutc(idx_weight);
%     successbin(k)=simoutc(1:w_consid)*weight(1:w_consid); %/(sum(simoutc)/length(simoutc))
%     failurebin(k)=1-successbin(k);
    
    [goal_confidence,z,z_ll,p_z] = general_oa_v2(datasample([0,1],500,'Weights',[failurebin(k) successbin(k)]), [-0.5,0.5,1], 2);
    goaest(k)= goal_confidence;
 
end

error_p2(1:length(goaest))=abs(goaest-oa(1:length(goaest))');
count(1:length(goaest))=count(1:length(goaest))+1;

end


temp_cutoff = max(find(count == Nsim))-1;


figure()
hold on;
grid on;
plot(error_p2(1:temp_cutoff)./count(1:temp_cutoff),'LineWidth',1)
xlabel('k');
ylabel('Mean GOA Error');
title('Mean Original vs. Phase 2 Estimate Error');

figure
plot(successbin)
xlabel('Time step (k)')
ylabel('Estimated Probability of Success')
ylim([0 1])
title('Particle Filter Probability Estimate')

figure
plot(goaest)
hold on
plot(oa)
ylim([-1 1])

% 

figure
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
a(2) = plot(1:length(goaest),goaest(1:end));
ylim([-1 1])
legend(a,{'Original','New'});
xlabel('k');
ylabel('GOA');

title('GOA value comparison');



%BS Stuff
N = 50;
c = 0.1;
for i = 1:N
        i
       [goaest,successbin,current,adversary,goal] = phase2_but_a_function(simcurr, simadv, simoutc, c);
        forecast_prob = zeros(1,length(goaest));
        actual_run = zeros(1,length(goaest));
        %Get a temporal BS for each step
        for w = 1:length(goaest)
          b = zeros(1,w);
          regret = zeros(1,w);
          regret2 = zeros(1,w);
          forecast_prob(w) = successbin(w);
          result =  runMCSims(workspace1,current(w,:),goal,adversary(w,:),1,0.4,inf);
          actual_run(w) = result(3);
        end
        b = (forecast_prob - actual_run).^2;

        for z = 1:length(actual_run)
            if actual_run(z) == 1
                regret(z) = ((1-goaest(z))/2)^2;
            else
                regret(z) = ((1 + goaest(z))/2)^2;

            end
        end
        bt(i) = mean(b);
        regrett(i) = mean(regret);
end
BS = sum(bt)/N;
R = sum(regrett)/N;

 

