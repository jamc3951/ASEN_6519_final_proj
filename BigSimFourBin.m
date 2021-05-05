clear;close all;clc;

%Define workspaces: obstacle code removed but simple c/p
plotting=0;
workspace1 = zeros(4,2,5);
O1 = [1 1; 2 1; 2 5; 1 5];
O2 = [3 4; 4 4; 4 12; 3 12];
O3 = [3 12; 12 12; 12 13; 3 13];
O4 = [12 5; 13 5; 13 13; 12 13];
O5 = [6 5; 12 5; 12 6; 6 6];
workspace1(:,:,1) = O1;
workspace1(:,:,2) = O2;
workspace1(:,:,3) = O3;
workspace1(:,:,4) = O4;
workspace1(:,:,5) = O5;


for i=1:1e6
    start = [normrnd(15,1,1), normrnd(0,1,1)];
    goal = [normrnd(10,1,1), normrnd(10,1,1)];
    enemy = [normrnd(5,6,1), normrnd(2,6,1)];
    
    d_0 = norm(start-goal);
    
    [current, adversary, ~, traj] = runMCSims3(workspace1,start,goal,enemy,1,.4,inf);
    
    [~,~,outcome] = composite_ordering(traj,d_0);
    simcurrp3{i}=current';
    simadvp3{i}=adversary';
    simoutcp3(i)=outcome';
    if plotting
        if outcome==1
            color='g-';
        else 
            color='r-';
        end
    
        figure(1)
        plot(current(:,2),adversary(:,2),color)
        hold on
        xlabel('Current y Position')
        ylabel('Adversary y Position')
        title('Simulated Outcomes')
    end
end


figure, histogram(simoutc)


%     Sh(i)=0;
%     for j=1:size(current_kp,1)
%         Sh(i)=Sh(i)+mvnpdf([current_c,adversary_c]',[current_kp(j,:),adversary_kp(j,:)]' ,10*eye(4))*outcomes_kp(j);
%     end
% 