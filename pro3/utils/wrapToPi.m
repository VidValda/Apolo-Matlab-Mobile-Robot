function y = wrapToPi(theta)
    y = mod(theta + pi, 2*pi) - pi;
end