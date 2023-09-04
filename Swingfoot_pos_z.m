function [pos] = Swingfoot_pos_z(lift_height, t_now, T, Tdsp)
    t1 = T * Tdsp / 2;
    t2 = T * (1 - Tdsp / 2);
    nt = t_now - t1;
    w = 2 * pi / (T * (1 - Tdsp));
              
    if t1 < t_now && t_now <= t2
        pos = 0.5 * lift_height * (1 - cos(w * nt));
    else 
        pos = 0;
    end
end