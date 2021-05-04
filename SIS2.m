function [w_k,current_kp,adversary_kp] = SIS2(N_s,C_s,workspace,goal,current,adversary,ykp,error)

current_kp = [];
adversary_kp = [];
outcomes_kp = [0,0];

%Characteristic Samples
for j = 1:C_s
    %Sample x_k+1
    [c,a,o,tp] = runMCSimsPF2(workspace,current,goal,adversary,1,error,Inf);
    
    outcomes(j) = o;
    current_kp(j,:) = c(2,:);
    adversary_kp(j,:) = a(2,:);
    transitions{j} = tp;
    
    chist{j}=c;
    ahist{j}=a;
    
    char_weights(j)=mvnpdf([current_kp(j,:),adversary_kp(j,:)],ykp,abs([current_kp(j,:),adversary_kp(j,:)]- ykp).*eye(4) + eye(4));
    
   
    if outcomes(j) == 1
        outcomes_kp(2) = outcomes_kp(2) + char_weights(j)*transitions{j}(2);
    else
        outcomes_kp(1) = outcomes_kp(1) + char_weights(j)*transitions{j}(2);
    end
    
end

char_weights=char_weights./sum(char_weights);
wkp=outcomes_kp./max(outcomes_kp);

t = sum(wkp);
wkp = wkp./(t);
w_k=wkp;

%weighted_mean = mean([current_kp(:,:),adversary_kp(:,:)].*char_weights',1);

outcomes_kp = [0,0];
for i = 1:(N_s-C_s)

    c = datasample(1:C_s,1,'Weights',char_weights); %sample by weights

    %P = ([current_kp(c,:),adversary_kp(c,:)] - weighted_mean)'*([current_kp(c,:),adversary_kp(c,:)] - weighted_mean); %-  weighted_mean*weighted_mean';
    
%     current_c = mvnrnd(current_kp(c,:),.05*eye(2)); 
    [current_c, tcc]= samplefollower(current,current_kp(c,:), .5, .01);
%     adversary_c = mvnrnd(adversary_kp(c,:),.05*eye(2));
    [adversary_c, taa]= samplefollower(adversary, adversary_kp(c,:), .5, .01);
    followerc(i,:)=current_c;
    followera(i,:)=adversary_c;
    
    Ps=5;
    Py=10;
    
%          weight = p_followertrans*p_meas/(p_transchar*p_sampfollow)   abs([current_c,adversary_c]- ykp).*eye(4) + 
        weight = 0.4*0.2*(mvnpdf([current_c,adversary_c],ykp,abs([current_c,adversary_c]- ykp).*eye(4) + eye(4)))/transitions{c}(2)/(tcc*taa);
%         weight = transitions{c}(2)*(mvnpdf([current_c,adversary_c],ykp,.25*eye(4)));
    if outcomes(c) == 1 %Variance?
        outcomes_kp(2) = outcomes_kp(2) + weight;
    else
        outcomes_kp(1) = outcomes_kp(1) +  weight;

    end
 
%     if outcomes(c) == 1 %Variance?
% 
%         outcomes_kp(2) = outcomes_kp(2) + char_weights(c)*mvnpdf([current_c,adversary_c],ykp,abs([current_c,adversary_c]- ykp).*eye(4) + eye(4)).*...
%             mvnpdf([current_c,adversary_c],[current,adversary],Ps*eye(4));
%     else
%         outcomes_kp(1) = outcomes_kp(1) +  char_weights(c)*mvnpdf([current_c,adversary_c],ykp,abs([current_c,adversary_c]- ykp).*eye(4) + eye(4)).*...
%             mvnpdf([current_c,adversary_c],[current,adversary],Ps*eye(4));
% 
%     end
end

% figure()
% hold on 
% scatter(current(1),current(2),'ob','LineWidth',1.5)
% hold on
% 
% scatter(followerc(:,1),followerc(:,2),'bx','LineWidth',1)
% axis square
% 
% colcod={'r','g'};
% for m=1:length(current_kp)
%     scatter(current_kp(m,1),current_kp(m,2),colcod{outcomes(m)+1},'LineWidth',1.5)
% end
% scatter(ykp(1),ykp(2),'sm','LineWidth',1.9)
% legend('Parent Pt','Followers','Characteristic Pt')



% scatter(goal(1),goal(2),'g*','LineWidth',1.2)
% scatter(adversary(1),adversary(2),'or')
% hold on
% scatter(adversary_kp(:,1),adversary_kp(:,2),'rx')

if N_s-C_s>0 
wkp = outcomes_kp./max(outcomes_kp);
t = sum(wkp);
wkp=wkp./t;
w_k = wkp+w_k;
% scatter(followerc(:,1),followerc(:,2),'yx')
% scatter(followera(:,1),followera(:,2),'yx')
end

w_k=w_k/sum(w_k);





    
end

