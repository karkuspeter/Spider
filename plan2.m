function [ u, V ] = plan( world, model, Pslip, V_init)
%value iteration
%   Detailed explanation goes here

Pslip = min(Pslip, 0.4);
probs = [1-2*Pslip; Pslip; Pslip];

if nargin <= 3
    V_init = zeros(size(world.r));
end
V = V_init;
Vprev = ones(size(world.r));
u = zeros(size(world.r));

actions = {[0 1], [0 -1], [1 0], [-1 0]};
diff = 1;
limit = 0.5;
counter = 0;

while diff > limit
    diff = max(max(abs(Vprev-V))); %update here, so after reaching limit one more iteration will be done
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
                
                for m=[3 2 1]
                    if coords(m, 1) < 1 || coords(m, 1) > size(world.r,1) || coords(m, 2) < 1 || coords(m,2) > size(world.r,2)
                        %probs(m) = 0;
                        probs(1) = probs(1) + probs(m);
                        probs(m) = 0;
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
    counter = counter+1;
end

%if counter > 60
%    [counter, Pslip]
%end

end