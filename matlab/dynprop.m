function [ql, xl, qf, xi] = dynprop(El, Fl, Ef, Ff, Ql, Qf, L, x0, xi0, w)

[~, ~, tau] = size(El);
[n2, n1, d] = size(L);

xl = zeros(n1, tau+1);
ql = zeros(n1, tau+1);
qf = zeros(n2, tau+1, d);
xi = zeros(n2, tau+1, d);


xl(:, 1) = x0; % initial condition for leader

for k = 1:d
    xi(:, 1, k) = xi0; % initial condition for follower under hypo k
end

ql(:, tau+1) = -Ql*w; % final condition for predicted follower co-state

for t = tau:-1:1
    ql(:, t) = El(:, :, t)'*ql(:, t+1) - Ql*w; % propagate leader's co-state

end

for t = 1:tau
    xl(:, t+1) = El(:, :, t)*xl(:, t) - Fl(:, :, t)*ql(:, t+1); % propogate leader's state
end


for k = 1:d
    qf(:, tau+1, k) = -Qf(:, :, k)*L(:, :, k)*xl(:, tau+1); % final condition for follower co-state under hypo k
end

for t = tau:-1:1
    for k = 1:d
        qf(:, t, k) = Ef(:, :, t, k)'*qf(:, t+1, k) - Qf(:, :, k)*L(:, :, k)*xl(:, t); % propagate follower co-state under hypothesis k
    end
end

for t = 1:tau
    for k = 1:d
        xi(:, t+1, k) = Ef(:, :, t, k)*xi(:, t, k) - Ff(:, :, t, k)*qf(:, t+1, k); % propogate follower's state
    end
end

end
