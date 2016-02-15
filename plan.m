function [ u, uraw, V, cpu_time] = plan( world, Pslip, policy0, R_add)
%value iteration
%   Detailed explanation goes here

Pslip = min(Pslip, 0.4);
if ~policy0
    policy0 = ones(size(world.Tnorm,1),1);
end

Pmat = world.Tnorm * (1-2*Pslip) + world.Tslip * Pslip;
Rmat = world.Rmat + ones(size(world.Rmat))*R_add;

%solve MDP
discount = 0.99;
[Vdisc, policy, iter, cpu_time] = mdp_policy_iteration(Pmat, Rmat, discount, policy0, 100,1);

% convert to u_plan
u = zeros(size(world.r));
uraw = policy;
V = u;
for i=1:size(world.r,1)
    for j=1:size(world.r,2)
        u(i,j) = policy(world.coord2mat(i,j));
        V(i,j) = Vdisc(world.coord2mat(i,j));
    end
end

end