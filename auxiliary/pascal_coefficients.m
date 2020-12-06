function coefs = pascal_coefficients(n)
% Compute the nth row of the pascal triangle
coefs = 1;
for i=1:n
    coefs = conv(coefs, [1 1]);
end
end

