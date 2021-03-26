function [collide] = checkCollision(workspace, current, adversary)
%For a given workspace, verify that the current position is not within a
%polygon (meaning collision)

%In this homework our primitives are strictly linear an univariante (x-4 =
%0) so we can take an approach like this

[l,d,o] = size(workspace);

if norm(current(end,:) - adversary(end,:)) < 0.25
    collide = 1;
else 
    collide = 0;
end

