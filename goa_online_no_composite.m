function [current, adversary, caught, pzs, oa] = goa_online_no_composite(workspace,start,goal,enemy,error,num_samples,horizon)

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
     [goal_confidence,z,z_ll,p_z] = general_oa_v2(traj(:,3), [-0.5,0.5,1], 2);
    oa = [oa; goal_confidence];
    pzs = [pzs;p_z];
    d2goal = distance(current(end,:), goal);
    [collide,current,adversary] = env_step(workspace,goal,current,adversary,d2goal,error,step);
    
    if collide == 1
        caught = [caught; 1];
        break;
    end
    caught = [caught;0];
end



end