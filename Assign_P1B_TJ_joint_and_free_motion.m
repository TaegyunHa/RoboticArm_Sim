%% Joint coordinates animation for task B2
% Taegyun Ha

%% Cear windows
close all
clc

%% Links Lengths
org = [0, 0, 0, 1]';
d1 = 0.063 ;
a2 = 0.153 ;
a3 = 0.153 ;
de = 0.098 ;                

%% Plot the workspace of the robot
figure (1)
set(1,'position',[560 190 560 420]) % Set a window position

xwork = zeros(360,360) ; % reserving space for the variables, because
ywork = zeros(360,360) ; % otherwise they would be created later within a loop.

%% End effector position

     EE_pos1 = [  0    1    0    0.067    ;
                  1   -0   -0   -0        ;
                 -0    0   -1    0.081    ;
                  0    0    0    1        ];
               
     EE_pos2 = [  0    1    0    0        ;
                  1   -0   -0    0.067    ;
                 -0    0   -1    0.081    ;
                  0    0    0    1        ];      
               

     EE_pos3 = [  0    1    0    0        ;
                  1   -0   -0    0.24     ;
                 -0    0   -1    0        ;
                  0    0    0    1        ];

     EE_pos4 = [  0    1    0    0        ;
                  1   -0   -0    0.067    ;
                 -0    0   -1    0.081    ;
                  0    0    0    1        ];

     EE_pos5 = [ -1    0    0   -0.067  ;
                  0    1    0    0      ;
                 -0    0   -1    0.081  ;
                  0    0    0    1     ];

     EE_pos6 = [ -1    0    0   -0.24  ;
                  0    1    0    0     ;
                 -0    0   -1    0     ;
                  0    0    0    1     ];
              
%% get q1, q2, q3, q4 from Position and orientation of EE
pos_ar1 = IK(EE_pos1);
pos_ar2 = IK(EE_pos3);
pos_ar3 = IK(EE_pos4);
pos_ar4 = IK(EE_pos5);
pos_ar5 = IK(EE_pos6);

%% Obtain arrays of q1, q2, q3, and q4 for each position
q1_array = [ pos_ar1(1) pos_ar2(1) pos_ar3(1) pos_ar4(1) pos_ar5(1) ];
q2_array = [ pos_ar1(2) pos_ar2(2) pos_ar3(2) pos_ar4(2) pos_ar5(2) ];
q3_array = [ pos_ar1(3) pos_ar2(3) pos_ar3(3) pos_ar4(3) pos_ar5(3) ];
q4_array = [ pos_ar1(4) pos_ar2(4) pos_ar3(4) pos_ar4(4) pos_ar5(4) ];

hold on
axis equal

%% Trajectory store array
tr_x = [];
tr_y = [];
tr_z = [];

q1_jc = [];
q2_jc = [];
q3_jc = [];
q4_jc = [];

%% Animation
steps = 80;

for i1 = 1 : (length(q1_array) - 1)
   for i2 = 1 : steps                

                q1 = q1_array(i1) + i2 * (q1_array(i1 + 1) - q1_array(i1))/steps;
                q2 = q2_array(i1) + i2 * (q2_array(i1 + 1) - q2_array(i1))/steps;
                q3 = q3_array(i1) + i2 * (q3_array(i1 + 1) - q3_array(i1))/steps;
                q4 = q4_array(i1) + i2 * (q4_array(i1 + 1) - q4_array(i1))/steps;
                
                q1_jc = [q1_jc, q1];
                q2_jc = [q2_jc, q2];
                q3_jc = [q3_jc, q3];
                q4_jc = [q4_jc, q4];
                
                T01 = distal(0    , pi/2, d1   , q1*pi/180);
                T12 = distal(a2   , 0   , 0    , q2*pi/180);
                T23 = distal(a3   , 0   , 0    , q3*pi/180);
                T34 = distal(0    , pi/2, 0    , q4*pi/180);
                T45 = distal(0    , 0   , de   , 0        );

                j1 = T01 * org;
                j2 = T01*T12 * org;
                j3 = T01*T12*T23 * org;
                j4 = T01*T12*T23*T34 * org;
                je = T01*T12*T23*T34*T45 * org;
                
                x = [j1(1) j2(1) j3(1) j4(1) je(1)]';
                y = [j1(2) j2(2) j3(2) j4(2) je(2)]';
                z = [j1(3) j2(3) j3(3) j4(3) je(3)]';
               
                % Draw All links of the robot arm
                plot3([0 j1(1)]' ,[0 j1(2)]', [0 j1(3)]', 'b', 'Linewidth', 4)
                axis equal
                %axis square
                grid on
                hold on
                plot3(x,y,z,'k', 'Linewidth', 1.5)
                
                % Draw Trajectory
                tr_x = [je(1), tr_x];
                tr_y = [je(2), tr_y];
                tr_z = [je(3), tr_z];
                traj = plot3(tr_x, tr_y, tr_z, '.r', 'MarkerSize', 0.01);
                
                % Draw Cylinder for base of robot
                [X,Y,Z] = cylinder(0.01);
                Z = Z*0.063;
                surf(X,Y,Z)
                
                % Set limit of x, y, and z axis
                xlim([-0.5 0.5])
                ylim([-0.5 0.5])
                zlim([ 0 0.5])
                
                pause(0.01)
                hold off
                
    end
end

%% Function for Distal
function result = distal( a_n, alph_n, d_n, th_n)

result = [      cos(th_n)        -cos(alph_n)*sin(th_n)    sin(alph_n)*sin(th_n)  a_n*cos(th_n);
                sin(th_n)         cos(alph_n)*cos(th_n)   -sin(alph_n)*cos(th_n)  a_n*sin(th_n);
                  0                   sin(alph_n)               cos(alph_n)            d_n;
                  0                       0                         0                   1      ];
end

%% Inverse Kinematics
function result = IK( EE_position )
% length
h  = 0.063 ;
l1 = 0.153 ;
l2 = 0.153 ;
l4 = 0.098 ;

% element
nx=  EE_position(1,1);
ny=  EE_position(2,1);
nz=  EE_position(3,1);
ox=  EE_position(1,2) ;
oy=  EE_position(2,2);
oz=  EE_position(3,2);
ax=  EE_position(1,3);
ay=  EE_position(2,3);
az=  EE_position(3,3);
px=  EE_position(1,4);
py=  EE_position(2,4);
pz=  EE_position(3,4);
 
% pwx, pwy, pwz, pwr, pr
 pwx = px - ax*l4;
 pwy = py - ay*l4;
 pwz = pz - az*l4;
 pwr = sqrt(pwx^2 + pwy^2);
 pr = sqrt(px^2 + py^2);
 
% q1, q2, q3, q4
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
 
 q_array = [i1 i2 i3 i4];
 result = q_array;
 
end

%% Function for r2d
function result = r2d(radian)
    result = radian*180/pi;
end
