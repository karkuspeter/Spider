x = 0.01:0.01:1;
figure()

% f = @(x, a, b, c)(a./x + b.^x + c);
% plot(x, 0.01./x + 1.8, x, 0.2./x + 0.4);
% %plot(x, f(x, 0.1, 10, 2).*(1-x)+(x).*f(x, 2, 0.5, 1));
% ylim([0 10]);


subplot(3,1,1);
f1 = @(x)(1*(1 - 0.9*gaussmf(x, [0.05, 0.3]) - 0.7*gaussmf(x, [0.03, 0.6])));
plot(x, f1(x));

slips = [0.1 0.5 0.9];

subplot(3,1,2);
%f2 = @(x,slip)(1-1./(1+exp(-(x-0.4)*15*slip)));
%plot(x, f2(x,slips(1)), x, f2(x,slips(2)), x, f2(x,slips(3)));

f2 = @(x,slip)(1-1./(1+exp(-(x-0.4)*15*slip)));
plot(x, f2(x,slips(1)), x, f2(x,slips(2)), x, f2(x,slips(3)));


subplot(3,1,3);
plot(x, f1(x) + f2(x,slips(1)), x, f1(x) + f2(x,slips(2)), x, f1(x) + f2(x,slips(3)));

