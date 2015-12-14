function [ u ] = plan( world, model, theta)
%value iteration
%   Detailed explanation goes here
Pslip = model.f(model.p, theta);

V = zeros(size(world.r));
Vprev = ones(size(world.r));
u = zeros(size(world.r));

actions = {[0 1], [0 -1], [1 0], [-1 0]};
diff = 1;
limit = 0.01;

while diff > limit
    diff = max(max(Vprev-V)); %update here, so after reaching limit one more iteration will be done
    Vprev = V;
    for i = 1:size(world.r,1)
        for j=1:size(world.r,2)
            if world.terminal(i,j)
                V(i,j) = 0;
                continue
            end
            values = zeros(4,1);
            for k=1:4
                move = cell2mat(actions(k));
                move_slip1 = move + [move(2) move(1)];
                move_slip2 = move - [move(2) move(1)];
                coords = [[i j] + move; [i j] + move_slip1; [i j] + move_slip2];
                probs = [1-2*Pslip; Pslip; Pslip];
                for l=1:3
                    if coords(l, 1) < 1 || coords(l, 1) > size(world.r,1) || coords(l, 2) < 1 || coords(l,2) > size(world.r,2)
                        probs(l) = 0;
                    end
                end
                if sum(probs) == 0
                    values(k) = -999999;
                    continue;
                end
                probs = probs / sum(probs);
                for l=1:3
                    if probs(l) > 0
                        values(k) = values(k) + probs(l)*(V(coords(l,1),coords(l,2)) + world.r(coords(l,1),coords(l,2)));
                    end
                end
            end
            [val dex] = max(values);
            V(i,j) = val;
            u(i,j) = dex;
        end
    end
end

end

