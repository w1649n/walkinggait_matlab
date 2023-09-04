function [com_vely0] = Com_vely0(x0, xt, px, T, Tc)
%x0: 初始位置
%xt: 下一時刻的位置
%px: 目標位置(ZMP)
%T : 步態週期
%Tc: 機器人自然週期
    com_vely0 = (xt - x0 * cosh(T / Tc) + (px) * (cosh(T / Tc) - 1)) / (Tc * sinh(T / Tc));
end