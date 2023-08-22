function [ f ] = vol(xi, Lam)
[~, tau1, N] = size(xi);

f = 0;

for t = 1:tau1
    for k1 = 1:N
        for k2 = k1+1:N
            W = Lam(:, :, t, k1) + Lam(:, :, t, k2);
            f = f + (xi(:, t, k1) - xi(:, t, k2))'*W*(xi(:, t, k1) - xi(:, t, k2));
        end
    end
end
end