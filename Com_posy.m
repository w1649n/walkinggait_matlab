function [com_posy] = Com_posy(x0, vx0, px, t, Tc)
%x0  : 初始位置
%xv0 : 初始速度
%px  : 目標位置(ZMP)
%t   : 當前時間
%T   : 步態週期
%Tc  : 機器人自然週期
    com_posy = px + x0 * cosh(t / Tc) + Tc * vx0 * sinh(t / Tc) - px * cosh(t / Tc);
end