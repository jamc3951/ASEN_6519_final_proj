function [states11,estates,tp,timeToCatch] = runMCSims3(workspace,start,goal,enemy,n,error,k)

plotting = 0;
timeToCatch = [];
%hold on;
for i = 1:n
    [states11,estates,caught,tp] = bug_tp(workspace,start,goal,enemy,error,k);
    timeToCatch = [timeToCatch; [length(states11) distance(states11(end,:),goal) caught]];
    
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

