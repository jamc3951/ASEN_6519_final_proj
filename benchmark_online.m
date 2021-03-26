function [p] = benchmark_online(base_label,Nl,error,workspace,k)
N = 100;
caught = zeros(1,N);
percentage = zeros(1,N);
p = zeros(1,length(Nl));
c = zeros(1,length(Nl));

for j = 1:length(Nl)
    for i = 1:N
        start = [rand*15, rand*15];
        goal = [rand*15, rand*15];
        enemy = [rand*15, rand*15];
        [agent2,enemy2,caught(i),pz,goa_inf] = goa_online(workspace,start,goal,enemy,error,Nl(j),k);
        correct = find(goa_inf > base_label(1));
        percentage(i) = length(correct)/length(goa_inf);
    end
    p(j) = mean(percentage);
end

figure()
grid on;
plot(Nl,p);
ylabel('Within initial estimate bounds');
xlabel('Number of samples at each step');
title('Arbitrary Configs');
end

