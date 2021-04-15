function [current, adversary, caught, pzs, oa] = goa_online(workspace,start,goal,enemy,error,num_samples,horizon)

current = [start];
adversary = [enemy];
oa = [];
pzs = [];
step = 0.5;
caught = [];
d_0 = distance(start,goal);
while norm(current(end,:) - goal) > step/2
    %Run MC sims
    traj = runMCSims(workspace,current(end,:),goal,adversary(end,:),num_samples,error,horizon);
    
    %Rank 2D
    [outcomes,bins,xBins] = composite_ordering(traj,d_0);
    
    %Compute OA
    d2goal = distance(current(end,:),goal);
    zstar = 4;
    [goa,~,pz] = goa_v3(outcomes,4,bins,xBins,zstar);
    oa = [oa; goa];
    pzs = [pzs;pz];
    
    [collide,current,adversary] = env_step(workspace,goal,current,adversary,d2goal,error,step);
    
    if collide == 1
        caught = [caught; 1];
        break;
    end
    caught = [caught;0];
end



end
