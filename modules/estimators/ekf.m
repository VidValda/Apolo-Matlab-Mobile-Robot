function [x_est,P] = ekf(x_est, u, z, dt, params)
    P = eye(3);
    x_est = [0;0;0];
end
