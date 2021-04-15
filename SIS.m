function [w_k,current_kp,adversary_kp] = SIS(N_s,workspace,goal,current,adversary,ykp,error)

current_kp = [];
adversary_kp = [];
outcomes_kp = [0,0];
for i = 1:N_s
    %Sample x_k+1
    [c,a,o] = runMCSimsPF(workspace,current,goal,adversary,1,error,Inf);
    
    outcomes(i) = o;
    current_kp(i,:) = c(2,:);
    adversary_kp(i,:) = a(2,:);
    
    if outcomes(i) == 1
        outcomes_kp(2) = outcomes_kp(2) + mvnpdf([current_kp(i,:),adversary_kp(i,:)],ykp(:)',10*eye(4));
    else
        outcomes_kp(1) = outcomes_kp(1) + mvnpdf([current_kp(i,:),adversary_kp(i,:)],ykp(:)',10*eye(4));
    end
end
t = sum(outcomes_kp);
wkp = outcomes_kp./(t);
w_k = wkp;
    
end

