%x = 1:100;
%y = 3*x + 10*rand(size(x));
%h = getFullGPModel(x,y);

n = 20;
x = gpml_randn(0.3, n, 1);
K = feval(covfunc{:}, hyp.cov, x);
mu = feval(meanfunc{:}, hyp.mean, x);
y = chol(K)'*gpml_randn(0.15, n, 1) + mu + exp(hyp.lik)*gpml_randn(0.2, n, 1);

%figure()
%plot(x, y, '+', 'MarkerSize', 12)


nlml = gp(hyp, @infExact, meanfunc, covfunc, likfunc, x, y);

figure()
z = linspace(-1.9, 1.9, 101)';
[m s2] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, x, y, z);

f = [m+2*sqrt(s2); flipdim(m-2*sqrt(s2),1)];
fill([z; flipdim(z,1)], f, [7 7 7]/8);

hold on;
plot(z, m, 'LineWidth', 2);
plot(x, y, '+', 'MarkerSize', 12)
axis([-1.9 1.9 -0.9 3.9])
grid on
xlabel('input, x')
ylabel('output, y')

%prob_vec = experience(:,2)./2./experience(:,3);
%theta_vec = experience(:,1);
x=[1; 2];
y=x+rand(size(x));
z = linspace(-1.9, 10, 101)';

figure()

loghyp = getFullGPModel(x,y);
[M, V, K] = predictWithFullGPModel(loghyp, x, y, z);
m=M;
s2=V;

f = [m+2*sqrt(s2); flipdim(m-2*sqrt(s2),1)];
fill([z; flipdim(z,1)], f, [7 7 7]/8);

hold on;
plot(z, m, 'LineWidth', 2);
plot(x, y, '+', 'MarkerSize', 12)
%axis([-1.9 1.9 -0.9 3.9])
grid on
xlabel('input, x')
ylabel('output, y')
