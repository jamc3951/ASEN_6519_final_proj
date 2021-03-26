function [timeToCatch] = runArbMCSims(workspace,n,error,k)
plotting = 0;
timeToCatch = [];
%hold on;
for i = 1:n
    start = [rand*15, rand*15];
    goal = [rand*15, rand*15];
    enemy = [rand*15, rand*15];
    [states11,estates,goal_achieved] = bug1(workspace,start,goal,enemy,error,k);
    timeToCatch = [timeToCatch; [length(states11) distance(states11(end,:),goal) goal_achieved]];
    
    if plotting ==1
        state_plot1= plot(states11(:,1),states11(:,2),'b','LineWidth',5);
        state_plot1.Color(4) = 0.03;
        state_plot2= plot(estates(:,1),estates(:,2),'r','LineWidth',5);
        state_plot2.Color(4) = 0.03;
    end
end
%title('Bugs - Workspace 1');
%xlabel('X');
%ylabel('Y');


end

