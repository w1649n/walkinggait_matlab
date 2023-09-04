function com_velx = com_velx(x0,xv0,px,t,T,C,q)
%x0:  初始位置
%xv0: 初始速度
%px:  目標位置(ZMP)
%t : 當前時間
%T : 步態週期
%Tc: 機器人自然週期
com_velx = x0*sinh(t/T)/T+xv0*cosh(t/T)-(px/T+(T*q/C^2))*sinh(t/T);
end

