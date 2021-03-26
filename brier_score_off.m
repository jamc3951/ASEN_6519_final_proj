function [BS,R] = brier_score_off(Nl,error,workspace,bins,z_star,k)
N = 150;
BS = [];
R = [];
for j = 1:length(Nl)
    b = zeros(1,N);
    regret = zeros(1,N);
    for i = 1:N
        start = [rand*15, rand*15];
        goal = [rand*15, rand*15];
        enemy = [rand*15, rand*15];
        traj =  runMCSims(workspace,start,goal,enemy,Nl(j),error,k);
        [goal_confidence,z,z_ll,p_z] = general_oa_v2(traj(:,3), bins, z_star);
        %Composite Ordering
        forecast_prob = sum(traj(:,3))/size(traj,1);
        actual_run =  runMCSims(workspace,start,goal,enemy,1,error,inf);
        b(i) = (forecast_prob - actual_run(3))^2;
        if actual_run(3) == 1
            regret(i) = ((1-goal_confidence)/2)^2;
        else
            regret(i) = ((1 + goal_confidence)/2)^2;
        end
    end
    BS(j) = sum(b)/N;
    R(j) = sum(regret)/N;
end

% figure()
% grid on;
% hold on;
% plot(Nl,BS);
% plot(Nl,R);
% ylabel('BS');
% xlabel('Number of samples');
% legend('BS', 'Regret')
% title('Offline BS w/ arb. configs');


end

