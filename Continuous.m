for time = 0 : sample_time_ : All_step * period_t_ - sample_time_
    Time(k) = time;
    t_ = mod(time, period_t_) + sample_time_;     %步週期內時刻(s)
    now_step_ = fix((time / period_t_) + 0.00000001); 
    var_theta_ = theta_(now_step_ + 1) / 180 * pi;

    if now_step_ < -2
        walking_state = "start";
    elseif now_step_ == 0
        walking_state = "first";
    elseif now_step_ == All_step - 1
        walking_state = "stop";
    else
        walking_state = "continuous";
    end

    if pre_step_ ~= now_step_
        if pre_step_ == -1
            footstep_x = 0;
            footstep_y = -width_size_;
            now_right_x_ = footstep_x;
            now_right_y_ = -width_size_;
            now_left_x_ = 0;
            now_left_y_ = width_size_;
        elseif mod(pre_step_,2)==1
            now_right_x_ = footstep_x;
            now_right_y_ = footstep_y;
        elseif mod(pre_step_,2)==0
            now_left_x_ = footstep_x;
            now_left_y_ = footstep_y;
        end

        zmp_x = footstep_x;
        zmp_y = footstep_y;
        last_displacement_x = displacement_x;   %上次的跨幅
        last_base_x = base_x;                   %上次到達的位置
        last_displacement_y = displacement_y;   %上次的Y軸位移量
        last_base_y = base_y;                   %上次的Y軸位移位置
        last_theta_ = var_theta_;               %前一次的Theta變化量

        if walking_state == "start"
            var_theta_ = 0;

            now_width = 2 * width_size_ * (-power(-1, now_step_ + 1));
            width_x = -sin(var_theta_) * now_width;
            width_y = cos(var_theta_) * now_width;
            displacement_x = width_x;
            displacement_y = width_y;
            footstep_x = footstep_x + width_x;
            footstep_y = footstep_y + width_y;
        elseif walking_state == "stop"
            now_width = 2 * width_size_ * (-power(-1, now_step_ + 1));
            width_x = -sin(var_theta_) * now_width;
            width_y = cos(var_theta_) * now_width;
            displacement_x = width_x;
            displacement_y = width_y;
            footstep_x = footstep_x + width_x;
            footstep_y = footstep_y + width_y;
        else
            now_width = 2 * width_size_ * (-power(-1, now_step_ + 1));
            width_x = -sin(var_theta_) * now_width;
            width_y = cos(var_theta_) * now_width;
            displacement_x = step_length_(now_step_ + 1) * cos(var_theta_) - shift_length_(now_step_ + 1) * sin(var_theta_) + width_x;
            displacement_y = step_length_(now_step_ + 1) * sin(var_theta_) + shift_length_(now_step_ + 1) * cos(var_theta_) + width_y;
            footstep_x = footstep_x + displacement_x;
            footstep_y = footstep_y + displacement_y;
        end

        base_x = (footstep_x + zmp_x) / 2;
        base_y = (footstep_y + zmp_y) / 2;
        
    end
    
    aa(k)  = last_base_x;
    aaa(k) = base_x;
    pre_step_ = now_step_;

    switch walking_state
        case "start"
            Cvx0(k) = Com_velx0(0, 0, zmp_x, period_t_, Tc_,C,q);
            Cvx(k) = Com_velx(0,Cvx0(k),0,t_,Tc_,C,q);
            Cpx(k) = Com_posx(0, Cvx0(k), zmp_x, t_, Tc_,C,q);

            Cvy0(k) = Com_vely0(0, 0, zmp_y, period_t_, Tc_);

            Cpz(k) = com_z_height;
            if mod(now_step_, 2) == 0
                Cpy(k) = Com_posy(0, Cvy0(k), zmp_y, t_, Tc_) + com_y_swing * sin(pi * t_ / period_t_);

                Lx(k) = Swingfoot_pos_XY(now_left_x_, 0, t_, period_t_, Tdsp);
                Ly(k) = Swingfoot_pos_XY(now_left_y_, 0, t_, period_t_, Tdsp);
                Lz(k) = Swingfoot_pos_z(lift_height_, t_, period_t_, Tdsp);

                Rx(k) = zmp_x;
                Ry(k) = zmp_y;
                Rz(k) = 0;
            elseif mod(now_step_,2)==1
                Cpy(k) = Com_posy(0, Cvy0(k), zmp_y, t_, Tc_);

                Lx(k) = zmp_x;
                Ly(k) = zmp_y;
                Lz(k) = 0;

                Rx(k) = Swingfoot_pos_XY(now_right_x_, 0, t_, period_t_, Tdsp);
                Ry(k) = Swingfoot_pos_XY(now_right_y_, 0, t_,period_t_, Tdsp);
                Rz(k) = Swingfoot_pos_z(lift_height_, t_, period_t_, Tdsp);
            end
        case "first"
            Cvx0(k) = Com_velx0(0, base_x, zmp_x, period_t_, Tc_,C,q);
            Cvx(k) = Com_velx(0,Cvx0(k),zmp_x,t_,Tc_,C,q);
            Cpx(k) = Com_posx(0, Cvx0(k), zmp_x, t_, Tc_,C,q);

            Cvy0(k) = Com_vely0(0, base_y, zmp_y, period_t_, Tc_);
            Cpy(k) = Com_posy(0, Cvy0(k), zmp_y, t_, Tc_);

            Cpz(k) = com_z_height;
            
            Lx(k) = Swingfoot_pos_XY(now_left_x_, displacement_x / 2, t_, period_t_, Tdsp);
            Ly(k) = Swingfoot_pos_XY(now_left_y_, (displacement_y - now_width) / 2, t_, period_t_, Tdsp);
            Lz(k) = Swingfoot_pos_z(lift_height_, t_, period_t_, Tdsp);

            Rx(k) = zmp_x;
            Ry(k) = zmp_y;
            Rz(k) = 0;
        case "stop"
            Cvx0(k) = Com_velx0(last_base_x, base_x, zmp_x, period_t_, Tc_,C,q);
            Cvx(k) = Com_velx(last_base_x,Cvx0(k),zmp_x,t_,Tc_,C,q);
            Cpx(k) = Com_posx(last_base_x, Cvx0(k), zmp_x, t_, Tc_,C,q);

            Cvy0(k) = Com_vely0(last_base_y, base_y, zmp_y, period_t_, Tc_);
            Cpy(k) = Com_posy(last_base_y, Cvy0(k), zmp_y, t_, Tc_);

            Cpz(k) = com_z_height;



            if mod(now_step_, 2) == 0
                Lx(k) = Swingfoot_pos_XY(now_left_x_, (last_displacement_x + displacement_x) / 2, t_, period_t_, Tdsp);
                Ly(k) = Swingfoot_pos_XY(now_left_y_, (last_displacement_y + displacement_y) / 2, t_, period_t_, Tdsp);
                Lz(k) = Swingfoot_pos_z(lift_height_, t_, period_t_, Tdsp);

                Rx(k) = zmp_x;
                Ry(k) = zmp_y;
                Rz(k) = 0;
            elseif mod(now_step_, 2) == 1
                Lx(k) = zmp_x;
                Ly(k) = zmp_y;
                Lz(k) = 0;

                Rx(k) = Swingfoot_pos_XY(now_right_x_, (last_displacement_x + displacement_x) / 2, t_, period_t_, Tdsp);
                Ry(k) = Swingfoot_pos_XY(now_right_y_, (last_displacement_y + displacement_y) / 2, t_, period_t_, Tdsp);
                Rz(k) = Swingfoot_pos_z(lift_height_, t_, period_t_, Tdsp);
            end
        case "continuous"
            Cvx0(k) = Com_velx0(last_base_x, base_x, zmp_x, period_t_, Tc_,C,q);
            Cvx(k) = Com_velx(last_base_x,Cvx0(k),zmp_x,t_,Tc_,C,q);
            Cpx(k) = Com_posx(last_base_x, Cvx0(k), zmp_x, t_, Tc_,C,q);

            Cvy0(k) = Com_vely0(last_base_y, base_y, zmp_y, period_t_, Tc_);
            Cpy(k) = Com_posy(last_base_y, Cvy0(k), zmp_y, t_, Tc_);

            Cpz(k) = com_z_height;

            if mod(now_step_, 2) == 0
                Lx(k) = Swingfoot_pos_XY(now_left_x_, (last_displacement_x + displacement_x) / 2,t_,period_t_ ,Tdsp);
                Ly(k) = Swingfoot_pos_XY(now_left_y_, (last_displacement_y + displacement_y) / 2,t_,period_t_ ,Tdsp);
                Lz(k) = Swingfoot_pos_z(lift_height_, t_,period_t_, Tdsp);

                Rx(k) = zmp_x;
                Ry(k) = zmp_y;
                Rz(k) = 0;
            elseif mod(now_step_, 2) == 1
                Lx(k) = zmp_x;
                Ly(k) = zmp_y;
                Lz(k) = 0;

                Rx(k) = Swingfoot_pos_XY(now_right_x_, (last_displacement_x + displacement_x) / 2, t_,period_t_, Tdsp);
                Ry(k) = Swingfoot_pos_XY(now_right_y_, (last_displacement_y + displacement_y) / 2, t_,period_t_, Tdsp);
                Rz(k) = Swingfoot_pos_z(lift_height_, t_, period_t_, Tdsp);
            end
    end
    % Cvx(k) = Com_velx(last_base_x,Cvx0(k),zmp_x,t_,Tc_,C,q);
    Cvy(k) = Com_vely(last_base_y,Cvy0(k),zmp_y,t_,Tc_,C,q);
    step_lxw(k) = Lx(k) - Cpx(k);
    step_rxw(k) = Rx(k) - Cpx(k);

    step_lyw(k) = Ly(k) - Cpy(k);
    step_ryw(k) = Ry(k) - Cpy(k);

    step_lx(k) = step_lxw(k) * cos(-var_theta_) - step_lyw(k) * sin(-var_theta_);
    step_ly(k) = step_lxw(k) * sin(-var_theta_) + step_lyw(k) * cos(-var_theta_);
    step_lz(k) = Cpz(k) - Lz(k);

    step_rx(k) = step_rxw(k) * cos(-var_theta_) - step_ryw(k) * sin(-var_theta_);
    step_ry(k) = step_rxw(k) * sin(-var_theta_) + step_ryw(k) * cos(-var_theta_);
    step_rz(k) = Cpz(k) - Rz(k);

    end_point_lx(k) = step_lx(k);
    end_point_ly(k) = step_ly(k) - Length_Pelvis / 2;
    end_point_lz(k) = step_lz(k) - (com_z_height - stand_height);
    
    end_point_rx(k) = step_rx(k);
    end_point_ry(k) = step_ry(k) + Length_Pelvis / 2;
    end_point_rz(k) = step_rz(k) - (com_z_height - stand_height);
    zmpx(k) = zmp_x;
    zmpy(k) = zmp_y;
    zmpz(k) = 0;
    k = k+1;    
end