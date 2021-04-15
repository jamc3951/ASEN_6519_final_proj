function [current, adversary, caught, pzs, oa, pf_oa] = goa_online_plusPF(workspace,start,goal,enemy,error,num_samples,horizon)
N_s = num_samples;
current = [start];
adversary = [enemy];
oa = [];
pf_oa = [];
pzs = [];
step = 0.5;
caught = [];
d_0 = distance(start,goal);
wk = zeros(1,N_s);


while norm(current(end,:) - goal) > step/2
    %Run MC sims
    traj = runMCSims(workspace,current(end,:),goal,adversary(end,:),num_samples,error,horizon);
    
    [goal_confidence,z,z_ll,p_z] = general_oa_v2(traj(:,3), [-0.5,0.5,1], 2);
    oa = [oa; goal_confidence];
    pzs = [pzs;p_z];
    d2goal = distance(current(end,:), goal);
    [collide,current,adversary] = env_step(workspace,goal,current,adversary,d2goal,error,step);
    
    %Pf here
    ykp = [current(end,:);adversary(end,:)];
    [w_k,current_kp,adversary_kp] = SIS(N_s,workspace,goal,current,adversary,ykp,error);
    [goal_confidence,z,z_ll,p_z] = general_oa_v2(datasample([0,1],N_s,'Weights',w_k), [-0.5,0.5,1], 2);
    pf_oa = [pf_oa; goal_confidence];
    
    if collide == 1
        caught = [caught; 1];
        break;
    end
    caught = [caught;0];
end




end

