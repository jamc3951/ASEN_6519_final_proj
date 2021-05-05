function [BS,R,R_pf] = brier_score_on3(Nl,Cs,error,workspace,z_star,k)
N = 50;
BS = [];
R = [];
R_pf = [];
for j = 1:length(Nl)
    j
    for i = 1:N
        
        start = [normrnd(15,.5,1), normrnd(0,.5,1)];
        goal = [normrnd(10,.5,1), normrnd(10,.5,1)];
        enemy = [normrnd(5,.5,1), normrnd(2,.5,1)];
        d_0 = distance(start,goal);
        %Get GOA-O for this 
        [agent2,enemy2,caught,p_z,goa_inf,pf_oa] = goa_online_pf3(workspace,start,goal,enemy,error,Nl(j),Cs(j),k,z_star);
        forecast_prob = zeros(1,length(goa_inf));
        actual_run = zeros(1,length(goa_inf));
        
        %Get a temporal BS for each step
        for w = 1:length(goa_inf)
          b = zeros(1,w);
          regret = zeros(1,w);
          regret2 = zeros(1,w);
          forecast_prob(w) = sum(p_z(w,z_star:end));
          [c,a,tp,traj] = runMCSims3(workspace,agent2(w,:),goal,enemy2(w,:),1,error,inf);
    
          [~,~,result] = composite_ordering(traj,d_0);
           if result >= z_star
               actual_run(w) = 1;
           else
               actual_run(w) = 0;
           end
        end
        b = (forecast_prob - actual_run).^2;

        for z = 1:length(actual_run)
            if actual_run(z) == 1
                regret(z) = ((1-goa_inf(z))/2)^2;
                regret2(z) = ((1-pf_oa(z))/2)^2;
            else
                regret(z) = ((1 + goa_inf(z))/2)^2;
                regret2(z) = ((1+pf_oa(z))/2)^2;
            end
        end
        bt(i) = mean(b);
        regrett(i) = mean(regret);
        regrett2(i) = mean(regret2);
    end
    BS(j) = sum(bt)/N;
    R(j) = sum(regrett)/N;
    R_pf(j) = sum(regrett2)/N;
end

figure()
grid on;
hold on;
plot(Nl,BS);
plot(Nl,R);
plot(Nl,R_pf);
ylabel('BS/RS');
xlabel('Number of samples');
legend('BS Original','RS Original','Rs New')
title('Online BS/RS Assessment, z(4)');


end
