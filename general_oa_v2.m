function [x0,z,z_ll,p_z] = general_oa_v2(x, xBinLEdges, z_star)
% *x = domain of ordered values, raw data
% z = domain of ranked outcome sets, 1:N (N=#bins/equiv classes for x)
% N = (z = N) -> best outcome on z -> (z = 1) worst set of outcomes
% *xBinLEdges: N ordered arbitrary bins over "true" x range (lower left
% edges: can include -Inf/+Inf)
% *z_star = threshold for which we care about bins above (inclusive)

%%Get histcounts via xBinLEdges:
[Ncx,Edgescx,xBin] = histcounts(x,xBinLEdges);

N = length(Ncx); %number of bins

if (z_star<0 || z_star>N)
    error('z_star must be integer between 1 and # of bins!')
end


%% Assign values to z domain (rank ordered outcomes, assuming xhistogram reflects desired order)
%%TODO (LATER): generalize for arbitrary xEquivalenceClass mapping to desired
%%ranked Z domain...
z = cell(1,N);
for kk = 1:N
   z{kk} = x(xBin==kk);   
end

%%estimate probabilities within each bin:
p_z = Ncx / sum(Ncx);

%%compute discrete LPM in z-space:
if z_star>1
    zL = 1:(z_star-1);
else
    zL = 1;
end
p_zL = p_z(zL);
DLPM_z = (z_star - zL)*p_zL';

%%compute discrete UPM in z-space (note: this should include z* index):
zU = z_star:N;
p_zU = p_z(zU);
DUPM_z = (zU - z_star + 1)*p_zU';

x0 = 2/(1+exp(-log(DUPM_z/DLPM_z))) - 1; %%compute OA self-confidence value

z_ll = Edgescx; %return lower limits of histogram mapping from x to z

end
