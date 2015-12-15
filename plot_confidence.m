function plot_confidence(x, y_mean, y_std)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

plot(x, y_mean)
f = [y_mean+2*y_std; flipdim(y_mean-2*y_std,1)];
fill([x; flipdim(x,1)], f, [7 7 7]/8);


end

