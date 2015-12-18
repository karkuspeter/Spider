function [hyp, xpseudo ] = optimizeGP(x, y0, M)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

[N,dim] = size(x);
M = min(N, M);

% initialize pseudo-inputs to a random subset of training inputs
[dum,I] = sort(rand(N,1)); clear dum;
I = I(1:M);
xb_init = x(I,:);

% initialize hyperparameters sensibly (see spgp_lik for how
% the hyperparameters are encoded)
hyp_init(1:dim,1) = -2*log((max(x)-min(x))'/2); % log 1/(lengthscales)^2
hyp_init(dim+1,1) = log(var(y0,1)); % log size 
hyp_init(dim+2,1) = log(var(y0,1)/4); % log noise

% optimize hyperparameters and pseudo-inputs
w_init = [reshape(xb_init,M*dim,1);hyp_init];
[w,f] = minimize(w_init,'spgp_lik',-200,y0,x,M);
% [w,f] = lbfgs(w_init,'spgp_lik',200,10,y0,x,M); % an alternative
xpseudo = reshape(w(1:M*dim,1),M,dim);
hyp = w(M*dim+1:end,1);


end

