function [com_posx] = Com_posx(x0, vx0, px, t, Tc,C,q)
%x0:  初始位置
%xv0: 初始速度
%px:  目標位置(ZMP)
%t : 當前時間
%T : 步態週期
%Tc: 機器人自然週期
% com_pos = px + x0 * cosh(t / Tc) + Tc * vx0 * sinh(t / Tc) - px * cosh(t / Tc);
    com_posx = x0 * cosh(t / Tc) + Tc * vx0 * sinh(t / Tc) - (px+((Tc^2)*q/(C^2))) * (cosh(t / Tc)-1);
end