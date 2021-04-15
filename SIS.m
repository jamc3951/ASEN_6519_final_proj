function [w_k,current_kp,adversary_kp] = SIS(N_s,C_s,workspace,goal,current,adversary,ykp,error)

current_kp = [];
adversary_kp = [];
outcomes_kp = [0,0];

%Characteristic Samples
for j = 1:C_s
    %Sample x_k+1
    [c,a,o] = runMCSimsPF(workspace,current,goal,adversary,1,error,Inf);
    
    outcomes(j) = o;
    current_kp(j,:) = c(2,:);
    adversary_kp(j,:) = a(2,:);
    
    
    if outcomes(j) == 1
        outcomes_kp(2) = outcomes_kp(2) + mvnpdf([current_kp(j,:),adversary_kp(j,:)],ykp(:)',10*eye(4));
    else
        outcomes_kp(1) = outcomes_kp(1) + mvnpdf([current_kp(j,:),adversary_kp(j,:)],ykp(:)',10*eye(4));
    end
    
end

for i = 1:(N_s-C_s)
    c = randsample(C_s,1,true); %Could sample by weights somehow
    current_c = mvnrnd(current_kp(c,:),5*eye(2));
    adversary_c = mvnrnd(adversary_kp(c,:),5*eye(2));
 
    if outcomes(c) == 1 %Variance?
        outcomes_kp(2) = outcomes_kp(2) + mvnpdf([current_c,adversary_c],ykp(:)',10*eye(4));
    else
        outcomes_kp(1) = outcomes_kp(1) + mvnpdf([current_c,adversary_c],ykp(:)',10*eye(4));
    end
end
t = sum(outcomes_kp);
wkp = outcomes_kp./(t);
w_k = wkp;
    
end

