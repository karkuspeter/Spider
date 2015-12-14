% visualize

u_plan = plan(world, struct('p', 0.1, 'f', @(n,m) n), 0);

mat = world.r;           
imagesc(-mat);            %# Create a colored plot of the matrix values
colormap(flipud(gray));  %# Change the colormap to gray (so higher values are
                         %#   black and lower values are white)

textStrings = num2str(mat(:),'%0.0f');  %# Create strings from the matrix values
textStrings = strtrim(cellstr(textStrings));  %# Remove any space padding
[x,y] = meshgrid(1:size(world.r,1),1:size(world.r,2));   %# Create x and y coordinates for the strings
hStrings = text(x(:),y(:),textStrings(:),...      %# Plot the strings
                'HorizontalAlignment','center');
midValue = mean(mean(mat));  %# Get the middle value of the color range
textColors = repmat(mat(:) < midValue,1,3);  %# Choose white or black for the
                                             %#   text color of the strings so
                                             %#   they can be easily seen over
                                             %#   the background color
set(hStrings,{'Color'},num2cell(textColors,2));  %# Change the text colors

set(gca,'XTick',1:size(world.r,1),...                         %# Change the axes tick marks
        'XTickLabel',{1:100},...  %#   and tick labels
        'YTick',1:size(world.r,2),...
        'YTickLabel',{'A','B','C','D','E','F','G','H'},...
        'TickLength',[0 0]);
hold on
    
[xa, ya] = meshgrid([1:size(world.r,1)],[1:size(world.r,2)]); 
arrow = {[0 1], [0 -1], [1 0], [-1 0]};
planv = zeros(size(u_plan));
planu = planv;
for i=1:size(u_plan,1)
    for j=1:size(u_plan,2)
        if u_plan(i,j) == 0
            continue
        end
        val = cell2mat(arrow(u_plan(i,j)))/4;
        planu(i,j) = val(2);
        planv(i,j) = val(1);
        xa(i,j) = xa(i,j) - val(2);
        ya(i,j) = ya(i,j) - val(1);
    end
end


quiver(xa, ya, planu, planv, 0.5);

[R, t] = execute(world, x0, u_plan, 0);
R

R_vec = [];
for i=1:1000
    [R, t] = execute(world, x0, u_plan, sqrt(0.1*2));
    R_vec = [R_vec; R];
end
mean(R_vec)


R_vec = [];
for i=1:1000
    [R, t] = execute(world, x0, u_plan, sqrt(0.2*2));
    R_vec = [R_vec; R];
end
mean(R_vec)

