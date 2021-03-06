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

    
    char_weights(j)=mvnpdf([current_kp(j,:),adversary_kp(j,:)],ykp,abs([current_kp(j,:),adversary_kp(j,:)]- ykp).*eye(4) + eye(4));
    
   
    if outcomes(j) == 1
        outcomes_kp(2) = outcomes_kp(2) + char_weights(j);
    else
        outcomes_kp(1) = outcomes_kp(1) + char_weights(j);
    end
    
end
char_weights=char_weights./sum(char_weights);
%weighted_mean = mean([current_kp(:,:),adversary_kp(:,:)].*char_weights',1);

for i = 1:(N_s-C_s)

    c = datasample(1:C_s,1,'Weights',char_weights); %sample by weights

    %P = ([current_kp(c,:),adversary_kp(c,:)] - weighted_mean)'*([current_kp(c,:),adversary_kp(c,:)] - weighted_mean); %-  weighted_mean*weighted_mean';
    
    current_c = mvnrnd(current_kp(c,:),10*eye(2));
    adversary_c = mvnrnd(adversary_kp(c,:),10*eye(2));
    Ps=5;
    Py=10;

 
    if outcomes(c) == 1 %Variance?

        outcomes_kp(2) = outcomes_kp(2) + char_weights(c)*mvnpdf([current_c,adversary_c],ykp,abs([current_c,adversary_c]- ykp).*eye(4) + eye(4)).*...
            mvnpdf([current_c,adversary_c],[current,adversary],Ps*eye(4));
    else
        outcomes_kp(1) = outcomes_kp(1) +  char_weights(c)*mvnpdf([current_c,adversary_c],ykp,abs([current_c,adversary_c]- ykp).*eye(4) + eye(4)).*...
            mvnpdf([current_c,adversary_c],[current,adversary],Ps*eye(4));

    end
end
t = sum(outcomes_kp);
wkp = outcomes_kp./(t);
w_k = wkp;
    
end

