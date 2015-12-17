base = 0.01;
alpha_vec = [base*50 base*10 base*5 base*2 base base/2 base/5 base/10 base/50 base/100];

for aiter=1:size(alpha_vec,2)
    alpha = alpha_vec(aiter);
    repeat;
    title(num2str(alpha))
end
