function xdesired = QuadrotorReferenceTrajectory(t)
% 参考轨迹
    x =6*sin(t/3);
    y = -6*sin(t/3).*cos(t/3);
    z = 6*cos(t/3);
    phi = zeros(1,length(t));
    theta = zeros(1,length(t));
    psi = zeros(1,length(t));
    xdot = zeros(1,length(t));
    ydot = zeros(1,length(t));
    zdot = zeros(1,length(t));
    phidot = zeros(1,length(t));
    thetadot = zeros(1,length(t));
    psidot = zeros(1,length(t));
 
    xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
 
end
 