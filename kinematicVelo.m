function v_o = kinematicVelo(p,n)
% parameter
V = 1;
d = 0.2;
Kx = 1;
Ky = 1;

theta_now = p(3);

x_tilda = n(1) - p(1);
y_tilda = n(2) - p(2);


theta_desire = atan2(y_tilda,x_tilda);
theta_tilda = theta_desire - theta_now;
if (theta_desire > -pi && theta_desire < -pi/2) && (theta_desire > pi/2 && theta_desire < pi)
    theta_desire = (pi + theta_desire);
elseif (theta_desire > 0 && theta_desire < pi/2) && (theta_desire > - pi/2 && theta_desire < 0)
     theta_desire = (-pi + theta_desire);
end

x_dot_goal = V * cos(theta_desire);
y_dot_goal = V * sin(theta_desire);
% x_dot_goal = 1;
% y_dot_goal = 1;

if (abs(x_tilda) < 0.1)
    x_dot_goal = 0;
end
if (abs(y_tilda) < 0.1)
    y_dot_goal = 0;
end

velocity_new = cos(theta_now) * (x_dot_goal + Kx*x_tilda) + sin(theta_now) * (y_dot_goal * Ky*y_tilda);
rotationalvelocity_new = (-1/d)*sin(theta_now) * (x_dot_goal + Kx*x_tilda) + (1/d)*cos(theta_now) * (y_dot_goal * Ky*y_tilda);
% velocity_new = cos(theta_now) * (x_dot_goal + Kx*x_tilda) + sin(theta_now) * (y_dot_goal * Ky*y_tilda);
% rotationalvelocity_new = (-1/d)*sin(theta_now) * (x_dot_goal + Kx*x_tilda) + (1/d)*cos(theta_now) * (y_dot_goal * Ky*y_tilda);


if velocity_new > 1
    velocity_new = 1;
elseif velocity_new < -1
    velocity_new = -1;
end
if rotationalvelocity_new > pi/8
    rotationalvelocity_new = pi/8;
elseif rotationalvelocity_new < -pi/8
    rotationalvelocity_new = -pi/8;
end

v_o(1) = velocity_new;
v_o(2) = rotationalvelocity_new;
 






