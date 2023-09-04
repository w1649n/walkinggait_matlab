function com_vely = com_vely(y0,yv0,py,t,T,C,q)
%x0:  初始位置
%xv0: 初始速度
%px:  目標位置(ZMP)
%t : 當前時間
%T : 步態週期
%Tc: 機器人自然週期
com_vely = y0*sinh(t/T)/T+yv0*cosh(t/T)-(py/T)*sinh(t/T);
end

