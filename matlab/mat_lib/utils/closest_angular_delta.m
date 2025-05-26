% calculates a2 - a1 and accounts for angular discontiuity by choosing the value closest to zero
function delta = closest_angular_delta(a2, a1)
    opt = [a2 - a1 + 2*pi, ...
           a2 - a1 - 2*pi, ...
           a2 - a1];
    [~, idx] = min(abs(opt));
    delta = opt(idx);
end