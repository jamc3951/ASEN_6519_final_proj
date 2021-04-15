function [xk_new,wk] = RESAMPLE(xk,wk,N_s)

c = [wk(1)];
for i = 2:N_s
   c(i) = c(i-1) + wk(i); 
end
i = 1;
u = [rand*(1/N_s)];
for j = 1:N_s
    u(j) = u(1) + (1/N_s)*(j-1);
    while u(j) > c(i)
        i = i + 1;
    end
    xk_new(:,j) = xk(:,i);
    wk(j) = 1/N_s;
end

end