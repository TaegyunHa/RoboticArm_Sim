%% Inverse Kinematics for Task A3
% Taegyun Ha

%% Cear windows
close all
clc

%% Length of Links
h  = 0.063 ;
l1 = 0.153 ;
l2 = 0.153 ;
l4 = 0.068 ;

%% End-effector position and rotation    
EE_pos = [  -1    0    0   -0.24  ;
              0    1    0    0     ;
              0    0   -1    0     ;
              0    0    0    1     ];
          
%% Elements of end-effector position
nx=  EE_pos(1,1);
ny=  EE_pos(2,1);
nz=  EE_pos(3,1);
ox=  EE_pos(1,2) ;
oy=  EE_pos(2,2);
oz=  EE_pos(3,2);
ax=  EE_pos(1,3);
ay=  EE_pos(2,3);
az=  EE_pos(3,3);
px=  EE_pos(1,4);
py=  EE_pos(2,4);
pz=  EE_pos(3,4);
    
 %% pwx, pwy, pwz, pwr, pr
 pwx = px - ax*l4;
 pwy = py - ay*l4;
 pwz = pz - az*l4;
 pwr = sqrt(pwx^2 + pwy^2);
 pr = sqrt(px^2 + py^2);
 
 %% q1, q2, q3, q4
 q1 = atan2(py, px);
 
    D = -( (pwr^2 + (pwz - h)^2) - l1^2 - l2^2 ) / (2*l1*l2);
 q3_p = atan2(  sqrt(1-D^2), -D ); % +
 q3_n = atan2( -sqrt(1-D^2), -D ); % -
 
    theta_q2 = atan2((pwz - h), pwr);
    alpha_q2 = atan2((l2*sin(q3_n)), (l1 + l2*cos(q3_n)));
 q2 = theta_q2 - alpha_q2;
  
    theta_cp = atan2(  sqrt(1-D^2), D ); % +
    theta_cn = atan2( -sqrt(1-D^2), D ); % -
    theta_3 = atan2( pz - pwz, pr - pwr );
    theta_2 = pi - q2 - theta_cp;
    q4 = theta_3 + theta_2 + pi/2;

 i1 = r2d(q1);
 i2 = r2d(q2);
 i3 = r2d(q3_n);
 i4 = r2d(q4);
 
%% Function for r2d
function result = r2d(radian)
    result = radian*180/pi;
end