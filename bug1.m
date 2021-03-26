function [current, adversary, goal_achieved] = bug1(workspace,start,goal,enemy,error,k)
%Left-turning Bug1 in 2D space

current = [start];
adversary = [enemy];
step = 0.5;
caught = 0;
counter = 0;
goal_achieved = 0;
while (norm(current(end,:) - goal) > step/2) && (counter < k)
    counter = counter +1;
    %move towards goal
    d2goal = distance(current(end,:),goal);
    direction = (goal - current(end,:))/d2goal;
    positive = sign(goal - current(end,:));    
    %stocastic
    chance = rand;
    if chance <= error
        %move in a random direction
        direction = 2*rand(1,2) - 1;
    end
    
    current = [current; current(end,:) + direction*step];
    
    %Adversary motion
    d2agent = distance(current(end,:),adversary(end,:));
    direction = abs(adversary(end,:) - current(end,:))/d2agent;
    positive = sign(current(end,:)-adversary(end,:));   
    
    chance = rand;
    if chance <= error/2
        %move in a random direction
        direction = 2*rand(1,2) - 1;
    end
    adversary = [adversary; adversary(end,:) + positive.*direction*step];
    
    collide = checkCollision(workspace,current,adversary);
    if collide == 1
        goal_achieved = 0;
        break;
    end
end

if norm(current(end,:) - goal) < step/2
    goal_achieved = 1;
else
    goal_achieved = 0;
end
end

