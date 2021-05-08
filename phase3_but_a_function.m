function [goaest,outcomebin,current,adversary,goal] = phase3_but_a_function(simcurrp3, simadvp3, simoutcp3, c)

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


Nsim = 1;
for sim = 1:Nsim

start = [normrnd(15,.5,1), normrnd(0,.5,1)];
goal = [normrnd(10,.5,1), normrnd(10,.5,1)];
enemy = [normrnd(5,.5,1), normrnd(2,.5,1)];

d_0 = norm(start-goal);

[current, adversary, goal_achieved] = bug1(workspace1,start,goal,enemy,.4,inf);

% [current, adversary, caught, pzs, oa]=goa_online_no_composite(workspace1,start,goal,enemy,.4,1000,inf);



outcomebin=zeros(4,length(current));
closeness = 0.05;


goaest=[];
w_consid=length(simcurrp3); % Start with considering whole dataset
weight=ones(w_consid,1)./(w_consid);
oa=[];
for k=1:length(current)-1

    traj = runMCSims(workspace1,current(k,:),goal,adversary(k,:),2000,.4,inf);
    [outcomes,bins,xBins] = composite_ordering(traj,d_0);
    [goa,~,p_z] = goa_v3(outcomes,4,bins,xBins,4);
    oa(k)=goa;


    
    for j=1:w_consid
        if length(simcurrp3{j})>= k
            weight(j)=mvnpdf([(simcurrp3{j}(:,k))', (simadvp3{j}(:,k))'], [current(k,:), adversary(k,:)], 1*eye(4));
            outcomebin(simoutcp3(j),k)=outcomebin(simoutcp3(j),k)+weight(j);
            
        else
            weight(j)=0;
        end
    end

    
    weight=weight./sum(weight);
    sbt= sum(outcomebin(:,k));
    if sbt == 0
        sim = sim-1;
        break
    end
    
    outcomebin(:,k)=outcomebin(:,k)/sbt;
    
    
    
    [sort_weight,idx_weight]=sort(weight,'descend');
    keepidx=find(sort_weight>=max(sort_weight)*closeness);
    
    weight=sort_weight;
    w_consid=length(keepidx);
    simcurrp3=simcurrp3(idx_weight);
    simadvp3=simadvp3(idx_weight);
    simoutcp3=simoutcp3(idx_weight);
%     successbin(k)=simoutcp3(1:w_consid)*weight(1:w_consid); %/(sum(simoutcp3)/length(simoutcp3))
%     failurebin(k)=1-successbin(k);
    
    [goal_confidence,z,z_ll,p_z] = general_oa_v2(datasample(0:3,500,'Weights',outcomebin(:,k)'), [-0.5,0.5,1.5,2.5,3.5], 4);
    goaest(k)= goal_confidence;
 
end

error_p2(1:length(goaest))=abs(goaest-oa(1:length(goaest)));
%count(1:length(goaest))=count(1:length(goaest))+1;

end

end