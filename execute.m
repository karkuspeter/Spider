function [ R, transitions ] = execute(world, x0, u_plan, theta)
%simulate experience
%   Detailed explanation goes here
slip_fun = @(theta)min(sum(theta.^2/2/length(theta)), 0.4);
Pslip = slip_fun(theta);

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
    R = [R; world.r(coord(1), coord(2))];
end

R = sum(R);

end

