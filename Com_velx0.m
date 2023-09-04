function [com_velx0] = Com_velx0(x0, xt, px, T, Tc,C,q)
%% 
%x0: 初始位置
%xt: 下一時刻的位置
%px: 目標位置(ZMP)
%T : 步態週期
%Tc: 機器人自然週期
    com_velx0 = (xt - x0 * cosh(T / Tc) + (px+((Tc^2)*q/(C^2))) * (cosh(T / Tc) - 1)) / (Tc * sinh(T / Tc));
end