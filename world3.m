function [ world ] = world3()
%create spider world v1

world = struct('r', 0 * ones(5,5), 'terminal', zeros(5,5), 'Tnorm', [], ...
    'Tslip', [], 'Rmat', [], 'coord2mat', @(c)(0));

world.r(2:4, 2:4) = -10;
world.r(5,3) = 10;
world.terminal(5,3) = 1;

world.r(3:5, 1) = -200;
world.r(3:5, 3:4) = -200;
world.r(7, 2) = 50;
world.terminal = (world.r ~= -1);
world.r(4, 5:7) = -60;

num_states = size(world.r,1)*size(world.r, 2);
Pmat1 = zeros(num_states, num_states, 4);
Pmat2 = Pmat1;
Rmat = Pmat1;
actions = {[0 1], [0 -1], [1 0], [-1 0]};


coord2mat = @(c1,c2)((c1-1)*size(world.r,2) + c2);
%mat2coord = @(c)(quorem(c,size(world.r,1)));
coordinvalid = @(coord)(coord(1)<1 || coord(2) < 1 || coord(1) > size(world.r,1) || coord(2) > size(world.r,2));

world.coord2mat = coord2mat;

for i=1:size(world.r,1)
    for j=1:size(world.r,2)
        if(world.terminal(i,j))
            for k=1:size(actions,1)
                Pmat1(coord2mat(i,j),coord2mat(i,j),k) = 1;
                Pmat2(coord2mat(i,j),coord2mat(i,j),k) = 2;
                Rmat(coord2mat(i,j),coord2mat(i,j),k) = 0;
            end
            continue;
        end
        for k=1:size(actions,2)
            move_noslip = cell2mat(actions(k));
            move_slip1 = move_noslip + [move_noslip(2) move_noslip(1)];
            move_slip2 = move_noslip - [move_noslip(2) move_noslip(1)];
            moves = [move_noslip; move_slip1; move_slip2];
            if coordinvalid([i j] + move_noslip)
                Pmat1(coord2mat(i,j),coord2mat(i,j),k) = 1;
                Pmat2(coord2mat(i,j),coord2mat(i,j),k) = 2;
                Rmat(coord2mat(i,j),coord2mat(i,j),k) = -1000;
                continue;
            end
            coord = [i j] + move_noslip;
            Pmat1(coord2mat(i,j), coord2mat(coord(1),coord(2)), k) = 1;
            Rmat(coord2mat(i,j), coord2mat(coord(1),coord(2)), k) = ...
                world.r(coord(1), coord(2));
            for m=2:3
                coord = [i j] + moves(m,:);
                if coordinvalid(coord)
                    coord = [i, j] + move_noslip;
                    % must be valid now
                end
                Pmat2(coord2mat(i,j), coord2mat(coord(1),coord(2)), k) = ...
                    Pmat2(coord2mat(i,j), coord2mat(coord(1),coord(2)), k) + 1;
                Rmat(coord2mat(i,j), coord2mat(coord(1),coord(2)), k) = ...
                    world.r(coord(1), coord(2));
            end
        end
    end
end

world.Tnorm = Pmat1;
world.Tslip = Pmat2;
world.Rmat = Rmat;

end

