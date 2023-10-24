%% 四旋翼无人机
% clear;clc;
% 创建非线性MPC
nx = 12;
ny = 12;
nu = 4;
nlmpcobj = nlmpc(nx, ny, nu);
% 状态空间表达式
nlmpcobj.Model.StateFcn = "QuadrotorStateFcn";
% 雅可比矩阵 加速运算
nlmpcobj.Jacobian.StateFcn = @QuadrotorStateJacobianFcn;
% 固定随机种子
rng(0)
% 验证模型
validateFcns(nlmpcobj,rand(nx,1),rand(nu,1));
% 设置仿真参数
Ts = 0.1;                     
p = 18;
m = 2;
nlmpcobj.Ts = Ts;
nlmpcobj.PredictionHorizon = p;
nlmpcobj.ControlHorizon = m;
% 输入约束
nlmpcobj.MV = struct( ...
    Min={0;0;0;0}, ...
    Max={10;10;10;10}, ...
    RateMin={-2;-2;-2;-2}, ...
    RateMax={2;2;2;2} ...
    );
% 输出权重
nlmpcobj.Weights.OutputVariables = [1 1 1 1 1 1 0 0 0 0 0 0];
% 输入权重
nlmpcobj.Weights.ManipulatedVariables = [0.1 0.1 0.1 0.1];
% 输入变化率权重
nlmpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];
% 设置初始状态
x = [7;-10;0;0;0;0;0;0;0;0;0;0];
 
% 控制目标(保持无人机悬浮的平均值)
nloptions = nlmpcmoveopt;
nloptions.MVTarget = [4.9 4.9 4.9 4.9]; 
mv = nloptions.MVTarget;

% 设置标题
title('Simulation of Flight Tracking');

%% 开始仿真
% 仿真时间间隔
Duration = 20;
% 等待条
hbar = waitbar(0,"Simulation Progress");
% 记录上一时刻的驶入
lastMV = mv;
% 记录历史值用于绘图
xHistory = x';
uHistory = lastMV;
 
x_refHistory = zeros(1,12);


 
% 仿真循环
for k = 1:(Duration/Ts)
 
    % 设置参考值
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    yref = QuadrotorReferenceTrajectory(t);
    x_refHistory(k,:) = QuadrotorReferenceTrajectory(k*Ts);
    
    % 计算控制量
    xk = xHistory(k,:);
    [uk,nloptions,info] = nlmpcmove(nlmpcobj,xk,lastMV,yref',[],nloptions);
 
    % 存储数据
    uHistory(k+1,:) = uk';
    lastMV = uk;
 
    % 预测下一时刻的无人机 (MVs = uk) 
    ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk);
    [TOUT,XOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
 
    % 更新无人机状态
    xHistory(k+1,:) = XOUT(end,:);
 
    % 更新进度条
    waitbar(k*Ts/Duration,hbar);
end
 
% 关闭进度条
close(hbar)
 
%% 绘制无人机轨迹跟踪
plotDemo_Double(x_refHistory(:,1:6), xHistory(:,1:6))