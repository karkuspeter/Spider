function [ R, transitions ] = execute(world, x0, u_plan, theta)
%simulate experience
%   Detailed explanation goes here
slip_fun = @(theta)min(mean(theta.^2/2, 2), 0.4);
Pslip = slip_fun(theta);

%theta_reward_func = @(theta)min(0,sqrt(mean(abs(theta/2), 2))-0.5);
theta_reward_func = @(theta)min(0,sigmf(mean(abs(theta),2), [20 0.35])*0.3-0.5);


actions = {[0 1], [0 -1], [1 0], [-1 0]};
coord = x0;
transitions = [];
R = [];

while world.terminal(coord(1), coord(2)) == 0 && size(R,1) < 150
    move_good = cell2mat(actions(u_plan(coord(1), coord(2))));
    move_slip1 = move_good + [move_good(2) move_good(1)];
    move_slip2 = move_good - [move_good(2) move_good(1)];
    while 1
        p = rand();
        if p < Pslip
            move = move_slip1;
            slipped = 1;
        elseif p < 2*Pslip
            move = move_slip2;
            slipped = 1;
        else
            move = move_good;
            slipped = 0;
        end
        next_coord = coord + move_good;
        if (next_coord(1) < 1 || next_coord(1) > size(world.r,1) || next_coord(2) < 1 || next_coord(2) > size(world.r,2))
            'cant happen'
        end
        next_coord = coord + move;
        if ~(next_coord(1) < 1 || next_coord(1) > size(world.r,1) || next_coord(2) < 1 || next_coord(2) > size(world.r,2))
            break
        end
    
        
    end
    coord = next_coord;
    transitions = [transitions; slipped];
    R = [R; (world.r(coord(1), coord(2))+theta_reward_func(theta))];
end

% cheat: reproduce transitions to reflect real Pslip, rather than decreased
% due to edge of world
for i=1:length(transitions)
    transitions(i) = (rand()<2*Pslip);
end
R = sum(R);

end

