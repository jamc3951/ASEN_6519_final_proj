function [] = benchmark_online_error(workspace1)
errors = linspace(0.2,0.6,5);
Nl = [250,100,50,20,10,1];
figure()
grid on;
hold on;
for w = 1:length(errors)
    error = errors(w);
    % Get Sims
    traj = runArbMCSims(workspace1,1000,error,inf);

    % Compute GOA's
    [goal_confidence,z,z_ll,p_z] = general_oa_v2(traj(:,3), [-0.5,0.5,1], 2);
    disp(['Baseline Confidence of reaching goal: ', string(goal_confidence)]);

    % Composite
    %d_0 = distance(start,goal);
    [outcomes,bins,xBin] = composite_ordering(traj,10.66);
    [composite_confidence,~,pz] = goa_v3(outcomes,4,bins,xBin,4);
    disp(['Baseline Comp. Confidence of reaching goal: ', string(composite_confidence)]);

    % labels = {-1: "Very Bad", -0.5: "Bad", -0.1: "Fair", 0.1: "Good", 0.5: "Very good"}


    % Compare to Online Assessment
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Composite Label -- change to goal confidence due to arb. d0
    labels = [-1,-0.5,-0.1,0.1,0.5];
    lower = labels((labels < composite_confidence));
    upper = labels((labels > composite_confidence));
    base_label = [lower(1)] ; 

    % Accuracy 
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
            [agent2,enemy2,caught(i),goa_inf] = goa_online(workspace1,start,goal,enemy,error,Nl(j),inf);
            correct = find(goa_inf > base_label(1));
            percentage(i) = length(correct)/length(goa_inf);
        end
        p(j) = mean(percentage);
    end
    plot(Nl,p);
end


ylabel('Within initial estimate bounds');
xlabel('Number of samples at each step');
legend(string(errors));
title('Arbitrary Configs');

end

