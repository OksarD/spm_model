% wraps between -pi <= angle < pi
function wrap = wrap_rad(angle)
    if angle >= pi
        wrap = angle - pi*2;
    elseif angle < -pi
        wrap = angle + pi*2;
    else
        wrap = angle;
    end
end