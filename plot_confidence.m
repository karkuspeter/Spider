function plot_confidence(x, y_mean, y_std, func)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

if nargin < 4
   func = @(x)(x)
end

hold on

f = [func(y_mean+2*y_std); flipdim(func(y_mean-2*y_std),1)];
fill([x; flipdim(x,1)], f, [7 7 7]/8);

plot(x, func(y_mean))

end

