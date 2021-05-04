function [collide,current,adversary] = env_step(workspace,goal,current,adversary,d2goal,error,step)
    
    %move towards goal
    direction = (goal - current(end,:))/d2goal;
    positive = sign(goal - current(end,:));    
    %stocastic
    chance = rand;
    if chance <= error
        %move in a random direction
        direction = 2*rand(1,2) - 1;
        direction=direction/norm(direction);
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
        direction=direction/norm(direction);
    end
    adversary = [adversary; adversary(end,:) + positive.*direction*step];
    
    collide = checkCollision(workspace,current,adversary);

end

