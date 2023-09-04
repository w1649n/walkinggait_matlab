for time = 0 : sample_time_ : 2 * period_t_ - sample_time_
    Time(k) = time;
    t_ = mod(time, period_t_) + sample_time_;     %步週期內時刻(s)
    now_step_ = fix((time / period_t_) + 0.00001); 
    var_theta_ = 0;

    if now_step_ < 1
        walking_state = "first";
    else
        walking_state = "stop";
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

        if walking_state == "first"
            now_width = 2 * width_size_ * (-power(-1, now_step_ + 1));
            width_x = -sin(var_theta_) * now_width;
            width_y = cos(var_theta_) * now_width;
            displacement_x = step_length_(now_step_ + 1) * cos(var_theta_) - shift_length_(now_step_ + 1) * sin(var_theta_) + width_x;
            displacement_y = step_length_(now_step_ + 1) * sin(var_theta_) + shift_length_(now_step_ + 1) * cos(var_theta_) + width_y;
            footstep_x = footstep_x + displacement_x;
            footstep_y = footstep_y + displacement_y;
        elseif walking_state == "stop"
            now_width = 2 * width_size_ * (-power(-1, now_step_ + 1));
            width_x = -sin(var_theta_) * now_width;
            width_y = cos(var_theta_) * now_width;
            displacement_x = width_x;
            displacement_y = width_y;
            footstep_x = footstep_x + width_x;
            footstep_y = footstep_y + width_y;
        end

        base_x = (footstep_x + zmp_x) / 2;
        base_y = (footstep_y + zmp_y) / 2;
    end

    pre_step_ = now_step_;

    switch walking_state
        case "first"
            Cvx(k) = Com_vel0(0, base_x, zmp_x, period_t_, Tc_);
            Cpx(k) = Com_pos(0, Cvx(k), zmp_x, t_, Tc_);
            Cvy(k) = Com_vel0(0, base_y + com_y_swing, zmp_y, period_t_, Tc_);
            Cpy(k) = Com_pos(0, Cvy(k), zmp_y, t_, Tc_);
            
            
            Lx(k) = Swingfoot_pos_XY(now_left_x_, displacement_x / 2, t_, period_t_, Tdsp);
            Ly(k) = Swingfoot_pos_XY(now_left_y_, (displacement_y - now_width) / 2, t_, period_t_, Tdsp);            

            Rx(k) = zmp_x;
            Ry(k) = zmp_y;

            if displacement_x == 0
                foot_lift_height = 0;
            else
                foot_lift_height = ((Lx(k) - now_left_x_) / displacement_x) * board_height;
                com_lift_height = foot_lift_height;
            end

            Cpz(k) = com_z_height + com_lift_height;
            Lz(k) = Swingfoot_pos_z(lift_height_, t_, period_t_, Tdsp) + foot_lift_height;
            Rz(k) = 0;
        case "stop"
            Cvx(k) = Com_vel0(last_base_x, base_x, zmp_x, period_t_, Tc_);
            Cpx(k) = Com_pos(last_base_x, Cvx(k), zmp_x, t_, Tc_);
            Cvy(k) = Com_vel0(last_base_y + com_y_swing, base_y, zmp_y, period_t_, Tc_);
            Cpy(k) = Com_pos(last_base_y + com_y_swing, Cvy(k), zmp_y, t_, Tc_);
            
            Lx(k) = zmp_x;
            Ly(k) = zmp_y;

            Rx(k) = Swingfoot_pos_XY(now_right_x_, (last_displacement_x + displacement_x) / 2, t_, period_t_, Tdsp);
            Ry(k) = Swingfoot_pos_XY(now_right_y_, (last_displacement_y + displacement_y) / 2, t_, period_t_, Tdsp);
            
            if (last_displacement_x + displacement_x) == 0
                foot_lift_height = 0;
            else
                foot_lift_height = ((Rx(k) - now_right_x_) / (last_displacement_x + displacement_x)) * board_height;
                com_lift_height = foot_lift_height;
            end

            Cpz(k) = com_z_height + board_height;
            Lz(k) = board_height;
            Rz(k) = Swingfoot_pos_z(lift_height_, t_, period_t_, Tdsp) + foot_lift_height + right_z_shift * sin(pi * t_ / period_t_);
    end

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

    k = k+1;    
end