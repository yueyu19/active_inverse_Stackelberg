function [xl, qf, xi] = dynprop(Al, Bl, Ef, Ff, Qf, L, Omegl, xl0, xf0, ul)

[~, ~, tau, ~] = size(Ef);
[nf, nl, d] = size(L);

xl = zeros(nl, tau+1);
qf = zeros(nf, tau+1, d);
xi = zeros(nf, tau+1, d);


xl(:, 1) = xl0; % initial condition for leader

for k = 1:d
    xi(:, 1, k) = xf0; % initial condition for follower under hypo k
end

% size(ul)

for t = 1:tau
    xl(:, t+1) = Al*xl(:, t) + Bl*ul(:, t) + mvnrnd(zeros(nl, 1), Omegl)'; % propogate leader's state
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
        xi(:, t+1, k) = Ef(:, :, t, k)*xi(:, t, k) - Ff(:, :, t, k)*qf(:, t+1, k); % propogate follower's state mean
    end
end

end