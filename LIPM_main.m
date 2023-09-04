clear
clc
%% robot parameter set
com_z_height = 29.5;    %質心高度
stand_height = 23.5;    %站姿高度
Length_Pelvis = 9;      %腳寬
mode = 3;
q=3000; % 0~3Nm  10000 3000
C=1;
g = 980;                %重力常數

if mode == 1
    Tc_ = sqrt(com_z_height/g); %機器人的自然週期
elseif mode == 2
    M=3.05;
    m=0.95;
    l=14;
    Tc_ = sqrt((M*com_z_height+M*l+m*Pz/4)/((M+m/2)*g));
elseif mode == 3
    M=3.05;     %機器人上半身重量(身體+手)
    mt=0.2;     %機器人支撐腿之大腿重量
    mc=0.75;    %機器人支撐腿之小腿重量
    l=14;       %機器人上半身體一半的長度
    C  = sqrt((M*com_z_height+M*l+mc*com_z_height/16+9*mt*com_z_height/16));
    Tc_ = sqrt((M*com_z_height+M*l+mc*com_z_height/16+9*mt*com_z_height/16)/((M+mc/4+3*mt/4)*g));
end
sample_time_ =  0.03;  %單位時間(s)
%% command set
Mode      = 4;          %(1)continuous (2)LC_1 (3)LC_2
All_step  = 15;          %模擬步數
LC_step   = 4;          %上板步態的步數(第N步上板)
LC_count  = 0;
LC_first  = false;
LC_finish = false;
%% 固定步態
step_param  = 3.5;                  %前進量(cm)
shift_param = 0;                    %平移量(cm)
theta_param = 0;                    %旋轉量(deg)

step_length_ = step_param * ones(1, 100);
shift_length_ = shift_param * ones(1, 100);   
theta_ = theta_param * ones(1, 100);          
%% 變步態
% step_length_    = [1 1 1 1 1 1 1 1 1 1];
% shift_length_   = [0 0 0 0 0 0 0 0 0 0];
% theta_          = [0 0 0 0 0 0 0 0 0 0]; 

width_size_   = 4.5;     %走路開腳寬度(cm)
period_t_     = 0.6;     %步週期(s)
Tdsp          = 0.4;     %雙支撐時間 比例(0~1)
lift_height_  = 4;       %擺盪腳高度
com_y_swing   = 0;       %起步補償
board_height  = 3;       %上板高度(負的變下板)
right_z_shift = 0;
smooth_LC     = 0.4;    %平滑第一步的質心高度(越低越平滑)
%% init
now_step_ = 0;
pre_step_ = -1;
walking_state = "stop";
footstep_x = 0;
footstep_y = -width_size_;
now_right_x_ = 0;
now_right_y_ = 0;
now_left_x_ = 0;
now_left_y_ = 0;
zmp_x = 0;
zmp_y = 0;
last_displacement_x = 0;
displacement_x = 0;
last_base_x = 0;
base_x = 0;
last_displacement_y = 0;
displacement_y = 0;
last_base_y = 0;
base_y = 0;
last_theta_ = 0;
var_theta_ = 0;
foot_lift_height = 0;
com_lift_height = 0;
%% simulate
k = 1;  %count

switch Mode
    case 1
        run("Continuous.m");
    case 2
        run("LC_1.m");
    case 3
        run("LC_2.m");
    case 4
        run("Continuous_LC.m")
end
%% 畫圖
run("plot_figure.m");