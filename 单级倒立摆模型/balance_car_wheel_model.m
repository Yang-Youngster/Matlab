T = 0.002;
%离散时间
n = 10000;
%仿真步数
m = 0.857;
%平衡车重量，单位kg
g = 9.8;
%重力加速度，单位m/s2
l = 0.05;
%轮子中心到质心距离，单位m
J = m*l*l;



%转动惯量
kp_out = 5;
ki_out = 0.1;
kd_out = 0;
%角度pid参数





kp_in = 100;
ki_in = 1;
kd_in = 0;
%角速度pid参数

a_ref = zeros(n,1);
%角度期望，单位°

da_ref = zeros(n,1);
%角速度期望，单位rad/s

dda_ref = zeros(n,1);
%角加速度期望，单位rad/s2

u = zeros(n,1);
%系统输入，即轮子的加速度，单位m/s2

a = zeros(n,1);
%角度状态，单位°


da = zeros(n,1);
%角速度状态，单位rad/s


dda = zeros(n,1);
%角加速度状态，单位rad/s2


time = zeros(n,1);
%时间轴

a_integral = 0;
%角度误差积分

da_integral = 0;
%角速度误差积分

a_last_error = 0;
%角度上次误差

da_last_error = 0;
%角速度上次误差

for i = 1:n
    time(i) = (i-1)*T;
    a_ref(i) = 10 * sin(time(i));                           
    %给出期望值
    % --------------------------- 仿真计算 ---------------------------%
    
    a_error = a_ref(i) - a(i);
    %角度误差

    a_integral = a_integral + a_error;
    %积分    误差积分=累加
    a_diff = (a_error - a_last_error)/T;
    %微分    误差的微分=微分的定义
    a_output = kp_out * a_error + ki_out * a_integral * T + kd_out * a_diff;
    %角度PID计算,对应公式7 - 区别
    
    % --------------------------- 角度环 ---------------------------%
    
    da_ref(i) = a_output;
    %角速度期望
    da_error = da_ref(i) - da(i);
    %角速度误差
    da_integral = da_integral + da_error;
    %积分
    da_diff = (da_error - da_last_error)/T;
    %差分
    da_output = kp_in * da_error + ki_in * da_integral * T + kd_in * da_diff;
    %角速度PID计算,对应公式11  
    
    % --------------------------- 角速度环 ---------------------------%
    
    dda_ref(i) = da_output;
    %角加速度期望
    u(i) = (J*dda_ref(i) - m*g*l*sin(a(i)/180*pi)) / (m*l*cos(a(i)/180*pi));
    %计算系统输入,对应公式13
    
    % --------------------------- 控制解算 ---------------------------%
    
    if i == n
        break;
    end
    dda(i) = (m*g*l*sin(a(i)/180*pi) + m*l*cos(a(i)/180*pi)*u(i)) / J; 
    %状态方程，对应公式2
    da(i+1) = da(i) + dda(i) * T;
    %角速度计算
    a(i+1) = a(i) + da(i) * T / pi * 180;
    %角度计算
    % --------------------------- 被控对象 ---------------------------%
end

figure(1);
subplot(2,2,1);
plot(time,a,'r',time,a_ref,'g');
title('角度响应曲线');
legend('角度','角度期望');
xlabel('time/s');
ylabel('角度/°');
subplot(2,2,2);
plot(time,da,'r',time,da_ref,'g');
title('角速度响应曲线');
legend('角速度','角速度期望');
xlabel('time/s');
ylabel('rad/s');
subplot(2,2,3);
plot(time,dda,'r',time,dda_ref,'g');
title('角加速度曲线');
legend('角加速度','角加速度期望');
xlabel('time/s');
ylabel('rad/s^2');
subplot(2,2,4);
plot(time,u,'r');
title('系统输入曲线');
legend('U');
xlabel('time/s');
ylabel('m/s^2');
