function res = solve_quadratic(a, b, c)
    discriminant = b^2 - 4*a*c;
    sqrt_discriminant = cmath.sqrt(discriminant);
    root1 = (-b + sqrt_discriminant) / (2*a);
    root2 = (-b - sqrt_discriminant) / (2*a);
    res = [root1, root2];
end