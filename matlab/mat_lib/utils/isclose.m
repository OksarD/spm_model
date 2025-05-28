function log = isclose(a, b)
    tol=1e-4;
    if abs(a-b) < tol
        log = true;
    else
        log = false;
    end

