function [ f ] = vol(xi, W)
[~, tau1, N] = size(xi);

f = 0;

for t = 1:tau1
    for k1 = 1:N
        for k2 = k1+1:N
            f = f + (xi(:, t, k1) - xi(:, t, k2))'*(W(:, :, t, k1) + W(:, :, t, k2))*(xi(:, t, k1) - xi(:, t, k2));
        end
    end
end
end