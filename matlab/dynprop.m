function [r, x, q, xi] = dynprop(El, Fl, Ef, Ff, Ql, Qf, L, x0, xi0, w)

[~, ~, tau] = size(El);
[n2, n1, d] = size(L);

x = zeros(n1, tau+1);
r = zeros(n1, tau+1);
q = zeros(n2, tau+1, d);
xi = zeros(n2, tau+1, d);


x(:, 1) = x0; % initial condition for leader

for k = 1:d
    xi(:, 1, k) = xi0; % initial condition for follower under hypo k
end

r(:, tau+1) = -Ql*w; % final condition for predicted follower co-state

for t = tau:-1:1
    r(:, t) = El(:, :, t)'*r(:, t+1) - Ql*w; % propagate leader's co-state

end

for t = 1:tau
    x(:, t+1) = El(:, :, t)*x(:, t) - Fl(:, :, t)*r(:, t+1); % propogate leader's state
end


for k = 1:d
    q(:, tau+1, k) = -Qf(:, :, k)*L(:, :, k)*x(:, tau+1); % final condition for follower co-state under hypo k
end

for t = tau:-1:1
    for k = 1:d
        q(:, t, k) = Ef(:, :, t, k)'*q(:, t+1, k) - Qf(:, :, k)*L(:, :, k)*x(:, t); % propagate follower co-state under hypothesis k
    end
end

for t = 1:tau
    for k = 1:d
        xi(:, t+1, k) = Ef(:, :, t, k)*xi(:, t, k) - Ff(:, :, t, k)*q(:, t+1, k); % propogate follower's state
    end
end

end