function [p] = brier_score_on(Nl,error,workspace,z_star,k)
N = 50;
BS = [];
R = [];
for j = 1:length(Nl)
    for i = 1:N
        start = [rand*15, rand*15];
        goal = [rand*15, rand*15];
        enemy = [rand*15, rand*15];

        %Get GOA-O for this 
        [agent2,enemy2,caught,p_z,goa_inf] = goa_online_no_composite(workspace,start,goal,enemy,error,Nl(j),k);
        forecast_prob = zeros(1,length(goa_inf));
        actual_run = zeros(1,length(goa_inf));
        
        %Get a temporal BS for each step
        for w = 1:length(goa_inf)
          b = zeros(1,w);
          regret = zeros(1,w);
          forecast_prob(w) = p_z(w,z_star);
          result =  runMCSims(workspace,agent2(w,:),goal,enemy2(w,:),1,error,inf);
          actual_run(w) = result(3);
        end
        b = (forecast_prob - actual_run).^2;
        for z = 1:length(actual_run)
            if actual_run(z) == 1
                regret(z) = ((1-goa_inf(z))/2)^2;
            else
                regret(z) = ((1 + goa_inf(z))/2)^2;
            end
        end
        bt(i) = mean(b);
        regrett(i) = mean(regret);
    end
    BS(j) = sum(bt)/N;
    R(j) = sum(regrett)/N;
end

figure()
grid on;
hold on;
plot(Nl,BS);
%plot(Nl,R);
ylabel('BS');
xlabel('Number of samples');
legend('BS')
title('Online BS w/ arb. configs');


end
