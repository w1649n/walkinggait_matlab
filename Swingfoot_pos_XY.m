function [point] = Swingfoot_pos_XY(Star, length, t_now, T, Tdsp)
    w = 2 * pi / (T * (1 - Tdsp));
    t1 = T * Tdsp / 2;
    t2 = T * (1 - Tdsp / 2);
    nt = t_now - t1;
    
    if t_now > 0 &&t_now <= t1
        point = Star;
    elseif t_now > t1 && t_now <= t2
        point = 2 * length * (w * nt - sin(w * nt)) / (2 * pi) + Star;
    else
        point = 2 * length + Star;
    end    
end