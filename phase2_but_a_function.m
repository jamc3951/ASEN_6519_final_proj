function [goaest,successbin,current,adversary,goal] = phase2_but_a_function(simcurr, simadv, simoutc, c)

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

% [current, adversary, goal_achieved] = bug1(workspace1,start,goal,enemy,.4,inf);

[current, adversary, caught, pzs, oa]=goa_online_no_composite(workspace1,start,goal,enemy,.4,1000,inf);

goal_achieved = ~caught(end);
successbin=zeros(1,length(current));
failurebin=zeros(1,length(current));
closeness = 0.1;


%%
w_consid=length(simcurr); % Start with considering whole dataset
weight=ones(w_consid,1)./(w_consid);
for k=1:length(current)
    
    for j=1:w_consid
        if length(simcurr{j})>= k
            weight(j)=mvnpdf([(simcurr{j}(:,k))', (simadv{j}(:,k))'], [current(k,:), adversary(k,:)], 10*eye(4));
        else
            weight(j)=0;
        end
    end
    
    weight=weight./sum(weight);
    
    
    
    [sort_weight,idx_weight]=sort(weight,'descend');
    keepidx=find(sort_weight>=max(sort_weight)*c);
    
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

end

