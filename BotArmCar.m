% 初始条件
x = 0; y = 0; theta = 0;
v = 1; % m/s
omega = 0.5; % rad/s

% 时间步长和总时间
dt = 0.1; 
total_time = 10; 

% 模拟
for t = 0:dt:total_time
    x_dot = v * cos(theta);
    y_dot = v * sin(theta);
    theta_dot = omega;

    x = x + x_dot * dt;
    y = y + y_dot * dt;
    theta = theta + theta_dot * dt;

    % 可以在此处加入 plot 命令绘制轨迹
    plot(x, y, 'bo');
    hold on;
    pause(dt);
end

xlabel('X');
ylabel('Y');
title('Two-wheel Robot Trajectory');
grid on;