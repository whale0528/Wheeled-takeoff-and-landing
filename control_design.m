%% 纵向控制器参数设计


%设计取alpha=4(d),高度1km，速度0.4ma,平衡多偏取-4.5(d)
wd_pitch=15;
kesi_pitch=0.8;
Tr=1.2;

V_control=128;%设计点速度
qc=0.5*rho0*V_control^2;%设计动压

a24=-Cma*qc*S_ref*c_ref/Iyy;
a25=-Cmde*qc*S_ref*c_ref/Iyy;
a34=(CLa*qc*S_ref)/(m*V_control);

Kw_pitch=-2*kesi_pitch*wd_pitch/a25;
K_alpha=(((wd_pitch^2)-a24))/(2*kesi_pitch*wd_pitch);
Kn_pitch=-(2.2*g*(wd_pitch)^2)/(Tr*(((wd_pitch)^2)-a24)*V_control*a34);


%% 偏航通道控制器设计
b24=-Cnb*qc*S_ref*b_ref/Izz;
b27=-Cndr*qc*S_ref*b_ref/Izz;
b34=(D0-CSb*qc*S_ref)/(m*V_control);
b37=-CSdr*qc*S_ref/(m*V_control);
kesi_yaw=0.8;
Tr_yaw=1.2;
wd_yaw=15;
Kw_yaw  = -2*kesi_yaw*wd_yaw/b27;
K_beta  = (((wd_yaw)^2)-b24)/(2*kesi_yaw*wd_yaw);
Kn_yaw  =  (2.2*g*wd_yaw^2)/(Tr_yaw*((wd_yaw^2)-b24)*V_control*b34);


%% 滚转通道控制器参数设计
save("control_design.mat","Kw_pitch","K_alpha","Kn_pitch","Kw_yaw","K_beta","Kn_yaw");