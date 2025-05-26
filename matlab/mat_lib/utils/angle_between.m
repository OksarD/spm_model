function angle = angle_between(a, b)
    a_norm = norm(a);
    b_norm = norm(b);
    angle = acos(dot(a, b) / (a_norm * b_norm));
end