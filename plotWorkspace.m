function [] = plotWorkspace(start, goal, m_title,agent3,enemy3,oa)
%Build convex hulls to show workspace visually from matrix of vertices

hold on;
grid on;
axis equal;
title(m_title);
xlabel('X');
ylabel('Y');
plot(start(1),start(2),'b.','MarkerSize',10)
plot(goal(1),goal(2),'g.','MarkerSize',20)
plot(enemy3(1,1),enemy3(1,2),'r.','MarkerSize',10)
a = plot(agent3(:,1),agent3(:,2),'b','LineWidth',5);
a.Color(4) = 0.3;
b = plot(enemy3(:,1),enemy3(:,2),'r','LineWidth',5);
b.Color(4) = 0.3;

subplot(2,1,2)
plot(1:length(oa),oa);
grid on;
%title('GOA - Short Term Confidence');
xlabel('k');
ylabel('GOA');
%figure(2) = histogram(ttc(:,1));

end

