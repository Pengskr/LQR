clc
clear
close all
% load path_S.mat
load path_Circle.mat

%% 相关参数定义
dt = 0.1;
L = 2.9;
Q = 100*eye(3);
R = 2*eye(2);

%% 轨迹处理
% 定义参考轨迹
refPos = path;
refPos_x = path(:, 1);
refPos_y = path(:, 2);

% 计算航向角和曲率
diff_x = diff(refPos_x);
diff_x(end+1) = diff_x(end);
diff_y = diff(refPos_y);
diff_y(end+1) = diff_y(end);
refHeading = atan2(diff_y , diff_x);                   % 航向角
derivative1 = gradient(refPos_y) ./ abs(diff_x);       % 一阶导数 gradient(refPos_y)和diff_y几乎相同
derivative2 = del2(refPos_y) ./ abs(diff_x);           % 二阶导数
refK = abs(derivative2) ./ (1+derivative1.^2).^(3/2);  % 计算曲率

% 根据阿克曼转向原理，计算参考前轮转角
refPos_Delta = atan(L*refK);

% 参考速度
refSpeed = 40/3.6;

%% 主程序
% 车辆初始状态
x = refPos_x(1)+0.5;
y = refPos_y(1)+0.5;
yaw = refHeading(1) + 0.02;

v = 10;
Delta = 0;
idx = 1;

% 实际跟踪轨迹
pos_actual = [x, y];
yaw_actual = yaw;
v_actual = v;
Delta_actual = Delta;
idx_actual = idx;
latError_LQR = [];

% 参考纯跟踪算法引入积分调节可以减小静态误差
Ki = 0.9;           % 积分调节系数
Err_integ = 0;

% 绘制参考轨迹
figure
plot(refPos_x,refPos_y,'b', 'LineWidth', 2)
xlabel('纵向坐标 / m');
ylabel('横向坐标 / m');
grid on;
grid minor
axis equal
hold on 

sizeOfPath = length(refPos_x);
while idx < sizeOfPath
    % 寻找参考轨迹最近目标点
    idx = calc_target_index(x, y, refPos_x, refPos_y);  

    % LQR控制器
    [v_delta, Delta_delta, delta_r, latError] = LQR_control(idx,x,y,refSpeed,yaw,refPos_x,refPos_y,refHeading,refPos_Delta,refK,L,Q,R,dt);

    % 如果误差过大，退出循迹
    if abs(latError) > 2
        disp('误差过大，退出程序!\n')
        break
    end

    % 前轮转角
    Err_integ = Err_integ + latError * dt;
    Delta_delta = Delta_delta - Ki * Err_integ; % y_error定义为y-y_r 所以此处Ki前用负号

    % 更新状态
    [x,y,yaw,v,Delta] = update(x, y, yaw, v, v_delta, Delta_delta, dt, L, refSpeed, delta_r);

    % 保存每一步的实际量
    pos_actual(end+1,:) = [x,y];
    yaw_actual(end+1) = yaw;
    v_actual(end+1,:)  = v;
    Delta_actual(end+1)  = Delta;
    idx_actual(end+1) = idx;
    latError_LQR(end+1,:) =  [idx,latError];

end

% 画图
for i = 1:size(pos_actual,1)
    % 实际位置
    scatter(pos_actual(i,1), pos_actual(i,2),150, 'r.');
    % 实际航向
    quiver(pos_actual(i,1), pos_actual(i,2), cos(yaw_actual(i)), sin(yaw_actual(i)),0.5, 'm', 'LineWidth', 1);     % 实际航向
    pause(0.01)
end
legend('参考车辆轨迹', '实际行驶轨迹','实际航向')

figure, plot(latError_LQR(:, 2));
grid on;
grid minor
title("横向误差")
ylabel('横向误差 / m');


% 保存
path_LQR = pos_actual;
save path_LQR.mat path_LQR
save latError_LQR.mat latError_LQR


%% 子函数
function idx = calc_target_index(x, y, refPos_x, refPos_y)
    i = 1:length(refPos_x);
    dist = sqrt((refPos_x(i)-x).^2 + (refPos_y(i)-y).^2);
    [~, idx] = min(dist);
end

function [v_delta,Delta_delta,delta_r,latError] = LQR_control(idx, x, y, refSpeed, yaw, refPos_x, refPos_y, ...
    refPos_yaw, refPos_Delta, refPos_k, L, Q, R, dt)
    x_r = refPos_x(idx);
    y_r = refPos_y(idx);
    yaw_r = refPos_yaw(idx);
    delta_r = refPos_Delta(idx);

    x_error = x-x_r;
    y_error = y-y_r;
    yaw_error = yaw-yaw_r;

    % 根据百度Apolo，计算横向误差
%     latError = y_error*cos(yaw_r) - x_error*sin(yaw_r);
    latError = y_error*cos(yaw) - x_error*sin(yaw);

    % 将误差值赋值到状态量
    X(1,1) = x_error; 
    X(2,1) = y_error;  
    X(3,1) = yaw_error;

    % 由状态方程矩阵系数，计算K
    A = [1,  0,  -refSpeed*dt*sin(yaw_r);
         0,  1,  refSpeed*dt*cos(yaw_r);
         0,  0,  1];
    B = [dt * cos(yaw_r),    0;
        dt * sin(yaw_r),    0;
        dt * tan(delta_r)/L,  refSpeed*dt/(L * cos(delta_r)^2)];

    K = calcu_K(A, B, Q, R);

    % 获得速度误差量、前轮转角误差量两个控制量
    u = -K * X;  % 2行1列
    v_delta = u(1);      
    Delta_delta = u(2);
end

function K = calcu_K(A, B, Q, R)
    iter_max = 500;
    epsilon = 0.01;
    
    P_old = 0;
    for i = 1:iter_max
        P_new = A' * P_old * A - (A' * P_old * B) / (R + B' * P_old * B) *( B' * P_old * A) +Q; 
        if abs(P_new-P_old)<=epsilon
            break
        else
            P_old = P_new;
        end
    end

    P = P_new;
    K = (B' * P * B + R) \ (B' * P * A);  % 2行3列
end

function [x,y,yaw,v,Delta] = update(x, y, yaw, v, v_delta, Delta_delta, dt, L, refSpeed, refDelta)
    Delta = refDelta + Delta_delta;
    x = x + v * cos(yaw) * dt;
    y = y + v * sin(yaw) * dt;
    v = refSpeed + v_delta; 
    yaw = yaw + v / L * tan(Delta) * dt;   
end