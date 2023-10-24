%% 绘制双无人机轨迹
% 机身坐标
function plotDemo_Double(X1, X2)
     figure(1);hold on; grid on; view(3);axis([-20, 20, -20, 20, -10, 20]);
 
    disp('开始绘制')
    iter = size(X1,1);
    for i = 1:iter
        plotQuadrotor(X1(i,:),X2(i,:),1-isequal(i, iter));
        fprintf('%d 绘制中...\n', i)
        plot3(X1(1:i,1),X1(1:i,2),X1(1:i,3),'r*');
        plot3(X2(1:i,1),X2(1:i,2),X2(1:i,3),'b*');
    end
end
 
function plotQuadrotor(X1,X2,fcla) % 位姿参数
    x1 = X1(1); y1 = X1(2); z1 = X1(3);
    rx1 = X1(4);ry1 = X1(5);rz1 = X1(6);
    x2 = X2(1); y2 = X2(2); z2 = X2(3);
    rx2 = X2(4);ry2 = X2(5);rz2 = X2(6); 
    % 定义无人机的参数
    body_radius = 0.8; % 机身半径
    body_height = 0.5; % 机身高度
    wing_radius = 0.4; % 旋翼半径
    wing_height = 0.25; % 旋翼高度
    body_dis = 0.4; % 旋翼与机体距离
    wing_len = body_radius + wing_radius + body_dis; % 机翼展长
    % 外观参数
    wing_color = [1 0 0];
    body_color = [1 1 0];
    
    
    %% 绘制1号无人机
    T = getMatrix(x1,y1,z1,rx1,ry1,rz1); % 获得机身的齐次交换矩阵
    T1 = T*[1 0 0 wing_len;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1]; 
    T2 = T*[1 0 0 0;
          0 1 0 wing_len;
          0 0 1 0;
          0 0 0 1]; 
    T3 = T*[1 0 0 -wing_len;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1]; 
    T4 = T*[1 0 0 0;
          0 1 0 -wing_len;
          0 0 1 0;
          0 0 0 1];
    
    DrawCylinder(T(1:3,4), T(1:3,3), body_radius,body_height, body_color); % 机身
    DrawCylinder(T1(1:3,4), T1(1:3,3), wing_radius,wing_height, wing_color); % 旋翼A
    DrawCylinder(T2(1:3,4), T2(1:3,3), wing_radius,wing_height, wing_color); % 旋翼B
    DrawCylinder(T3(1:3,4), T3(1:3,3), wing_radius,wing_height, wing_color); % 旋翼C
    DrawCylinder(T4(1:3,4), T4(1:3,3), wing_radius,wing_height, wing_color); % 旋翼D
    
    % 连线
    connect(T,T1); 
    connect(T,T2);
    connect(T,T3);
    connect(T,T4);
 
 
    %% 绘制2号无人机
    T = getMatrix(x2,y2,z2,rx2,ry2,rz2); % 获得机身的齐次交换矩阵
    T1 = T*[1 0 0 wing_len;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1]; 
    T2 = T*[1 0 0 0;
          0 1 0 wing_len;
          0 0 1 0;
          0 0 0 1]; 
    T3 = T*[1 0 0 -wing_len;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1]; 
    T4 = T*[1 0 0 0;
          0 1 0 -wing_len;
          0 0 1 0;
          0 0 0 1];
    
    DrawCylinder(T(1:3,4), T(1:3,3), body_radius,body_height, body_color); % 机身
    DrawCylinder(T1(1:3,4), T1(1:3,3), wing_radius,wing_height, wing_color); % 旋翼A
    DrawCylinder(T2(1:3,4), T2(1:3,3), wing_radius,wing_height, wing_color); % 旋翼B
    DrawCylinder(T3(1:3,4), T3(1:3,3), wing_radius,wing_height, wing_color); % 旋翼C
    DrawCylinder(T4(1:3,4), T4(1:3,3), wing_radius,wing_height, wing_color); % 旋翼D
    
    % 连线
    connect(T,T1); 
    connect(T,T2);
    connect(T,T3);
    connect(T,T4);    
%     hold off;
    pic=getframe;
    if(fcla)
        cla;
    end
end
%%
%%%%%%%%%%%%%%%
%  函数定义    %
%%%%%%%%%%%%%%%
function T = getMatrix(x,y,z,rx,ry,rz) % 获取机身的齐次变换矩阵
    T_p = [1 0 0 x;
           0 1 0 y;
           0 0 1 z;
           0 0 0 1];
 
    T_x = [1 0 0 0;
           0 cos(rx) -sin(rx) 0;
           0 sin(rx) cos(rx) 0;
           0 0 0 1];
    T_y = [cos(ry)  0 sin(ry) 0;
           0 1 0 0;
          -sin(ry) 0 cos(ry) 0;
           0 0 0 1];
    T_z = [cos(rz) -sin(rz) 0 0;
        sin(rz) cos(rz)  0 0;
        0 0 1 0;
        0 0 0 1];
    T = T_p*T_z*T_y*T_x;
end
 
%% 绘制圆柱体
function h = DrawCylinder(pos, az, radius,len, col)
    
    az0 = [0;0;1];
    ax  = cross(az0,az);
    ax_n = norm(ax);
    if ax_n < eps 
	    rot = eye(3);
    else
        ax = ax/ax_n;
        ay = cross(az,ax);
        ay = ay/norm(ay);
        rot = [ax ay az];
    end
    
    %********** make cylinder
    % col = [0 0.5 0];  % cylinder color
    
    a = 50;    % number of side faces
    theta = (0:a)/a * 2*pi;
    
    x = [radius; radius]* cos(theta);
    y = [radius; radius] * sin(theta);
    z = [len/2; -len/2] * ones(1,a+1);
    % cc = col*ones(size(x));
    
    for n=1:size(x,1)
       xyz = [x(n,:);y(n,:);z(n,:)];
       xyz2 = rot * xyz;
       x2(n,:) = xyz2(1,:);
       y2(n,:) = xyz2(2,:);
       z2(n,:) = xyz2(3,:);
    end
    
    %************* draw
    % side faces
    h = surf(x2+pos(1),y2+pos(2),z2+pos(3),'FaceColor',col,'LineStyle','none');
    
    for n=1:2
	    patch(x2(n,:)+pos(1),y2(n,:)+pos(2),z2(n,:)+pos(3),col);
    end	
end
 
function connect(T1, T2)
    p1 = T1(1:3,4);
    p2 = T2(1:3,4);
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)],'color','black','LineWidth',5);
end