function [follower, t]=samplefollower(parent, char, step, sig)
dev=mvnrnd([0 0], sig*eye(2));
t = mvnpdf(dev, [0 0], sig*eye(2))/mvnpdf([0 0], [0 0], sig*eye(2));
dir=(char-parent)+dev;
dir=dir/norm(dir);

follower = parent + step*dir;




end