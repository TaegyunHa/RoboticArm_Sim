
%% Cear windows
close all
clc
disp('The following code simulates the FK')

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

%% Sampling the Trajectory( position1, position2, sample_num, v, a)
sample_numbers1 = 80;
sample_numbers2 = 140;

Trajectory0 = Traj( EE_pos1, EE_pos3, sample_numbers1, 0.01, 0.002);
Trajectory1 = Traj_Blend( EE_pos1, EE_pos2, EE_pos3, sample_numbers1, 0.01, 0.002);
Trajectory2 = Traj_Blend_2( EE_pos3, EE_pos4, EE_pos5, EE_pos6, sample_numbers2, 0.01, 0.002);

%% get q1, q2, q3, q4 from Position and orientation of EE
ar_pos = [];

% for i = 1 : sample_numbers1
%     temp_ar = [ Trajectory0(1,(1 + 4*(i-1))) Trajectory0(1,(2 + 4*(i-1))) Trajectory0(1,(3 + 4*(i-1))) Trajectory0(1,(4 + 4*(i-1))); 
%                  Trajectory0(2,(1 + 4*(i-1))) Trajectory0(2,(2 + 4*(i-1))) Trajectory0(2,(3 + 4*(i-1))) Trajectory0(2,(4 + 4*(i-1))); 
%                  Trajectory0(3,(1 + 4*(i-1))) Trajectory0(3,(2 + 4*(i-1))) Trajectory0(3,(3 + 4*(i-1))) Trajectory0(3,(4 + 4*(i-1))); 
%                  0                  0                   0                  1                ];
%      ar_pos = [ar_pos, IK(temp_ar)'];
% end

for i = 1 : sample_numbers1
     temp_ar = [ Trajectory1(1,(1 + 4*(i-1))) Trajectory1(1,(2 + 4*(i-1))) Trajectory1(1,(3 + 4*(i-1))) Trajectory1(1,(4 + 4*(i-1))); 
                 Trajectory1(2,(1 + 4*(i-1))) Trajectory1(2,(2 + 4*(i-1))) Trajectory1(2,(3 + 4*(i-1))) Trajectory1(2,(4 + 4*(i-1))); 
                 Trajectory1(3,(1 + 4*(i-1))) Trajectory1(3,(2 + 4*(i-1))) Trajectory1(3,(3 + 4*(i-1))) Trajectory1(3,(4 + 4*(i-1))); 
                 0                  0                   0                  1                ];
     ar_pos = [ar_pos, IK(temp_ar)'];
end

for i = 1 : sample_numbers2
     temp_ar = [ Trajectory2(1,(1 + 4*(i-1))) Trajectory2(1,(2 + 4*(i-1))) Trajectory2(1,(3 + 4*(i-1))) Trajectory2(1,(4 + 4*(i-1))); 
                 Trajectory2(2,(1 + 4*(i-1))) Trajectory2(2,(2 + 4*(i-1))) Trajectory2(2,(3 + 4*(i-1))) Trajectory2(2,(4 + 4*(i-1))); 
                 Trajectory2(3,(1 + 4*(i-1))) Trajectory2(3,(2 + 4*(i-1))) Trajectory2(3,(3 + 4*(i-1))) Trajectory2(3,(4 + 4*(i-1))); 
                 0                  0                   0                  1                ];
     ar_pos = [ar_pos, IK(temp_ar)'];
end

q1_array = [];
q2_array = [];
q3_array = [];
q4_array = [];

%% Store q1, q2, q3, q4 in seperate arrays for plotting
for i = 1 : length(ar_pos)
    q1_array = [ q1_array ar_pos(1,i) ];
    q2_array = [ q2_array ar_pos(2,i) ];
    q3_array = [ q3_array ar_pos(3,i) ];
    q4_array = [ q4_array ar_pos(4,i) ];
end

%% Declare Matriece to store trajectory
tr_x = [];
tr_y = [];
tr_z = [];

%% Animation
for i1 = 1 : length(q1_array)

                q1 = q1_array(i1);
                q2 = q2_array(i1);
                q3 = q3_array(i1);
                q4 = q4_array(i1);
                
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
               
                % Draw Base
                plot3([0 j1(1)]' ,[0 j1(2)]', [0 j1(3)]', 'b', 'Linewidth', 4) % Base
                axis equal
                % axis square
                grid on
                hold on
                
                % Draw Links of the robot arm
%                 plot3(x,y,z,'.-k', 'Linewidth', 1.5, 'MarkerSize', 15) % Links
                plot3(x,y,z,'k', 'Linewidth', 1.5) % Links
                
                % Draw Trajectory
                tr_x = [je(1), tr_x];
                tr_y = [je(2), tr_y];
                tr_z = [je(3), tr_z];
                traj = plot3(tr_x, tr_y, tr_z, '.r', 'MarkerSize', 0.01);
                
                % Draw Cylinder for base of robot
                [X,Y,Z] = cylinder(0.01);
                Z = Z*0.063;
                surf(X,Y,Z)
                
                % Draw Obstacle
                [X2,Y2,Z2] = cylinder(0.01);
                X2 = X2+0.055;
                Y2 = Y2+0.055;
                Z2 = Z2*0.1;
                surf(X2,Y2,Z2, 'FaceColor', 'r')
                
                % Set limit of x, y, and z axis
                xlim([-0.4 0.4])
                ylim([-0.4 0.4])
                zlim([ 0 0.4])
                
                pause(0.02)
                hold off
                

end





%% Function for Distal
function result = distal( a_n, alph_n, d_n, th_n)

result = [      cos(th_n)        -cos(alph_n)*sin(th_n)    sin(alph_n)*sin(th_n)  a_n*cos(th_n);
                sin(th_n)         cos(alph_n)*cos(th_n)   -sin(alph_n)*cos(th_n)  a_n*sin(th_n);
                  0                   sin(alph_n)               cos(alph_n)            d_n;
                  0                       0                         0                   1      ];
end

%% Sampling Trajectory
function result = Traj( position1, position2, sample_num, v, a)
%% Obtain components of start and end position
nx1=  position1(1,1);
ny1=  position1(2,1);
nz1=  position1(3,1);
ox1=  position1(1,2) ;
oy1=  position1(2,2);
oz1=  position1(3,2);
ax1=  position1(1,3);
ay1=  position1(2,3);
az1=  position1(3,3);
px1=  position1(1,4);
py1=  position1(2,4);
pz1=  position1(3,4);

nx2=  position2(1,1);
ny2=  position2(2,1);
nz2=  position2(3,1);
ox2=  position2(1,2) ;
oy2=  position2(2,2);
oz2=  position2(3,2);
ax2=  position2(1,3);
ay2=  position2(2,3);
az2=  position2(3,3);
px2=  position2(1,4);
py2=  position2(2,4);
pz2=  position2(3,4);

%% Obtain unit vector and length between two position
sample = [];
sample_time = [];
L = sqrt((px2 - px1)^2 + (py2 - py1)^2 + (pz2 - pz2)^2);
u = [(px2 - px1)/L (py2 - py1)/L (pz2 - pz1)/L]';
Tb = v/a;
T1 = (L*a + v^2)/(a*v);

a_x1 = a * u(1);
a_y1 = a * u(2);
a_z1 = a * u(3);
v_x1 = v * u(1);
v_y1 = v * u(2);
v_z1 = v * u(3);

% Sample
    for t = linspace(0, T1, sample_num)
        sample_time = [sample_time, t];
        
        if t <= Tb
                px_s = px1 + (a_x1 * t^2) / 2;
                py_s = py1 + (a_y1 * t^2) / 2;
                pz_s = pz1 + (a_z1 * t^2) / 2;
                sigma = [ px_s;
                          py_s;
                          pz_s];
       
        elseif t > Tb && t < (T1 - Tb)
                px_s = px1 + v_x1*t - a_x1*Tb^2/2;
                py_s = py1 + v_y1*t - a_y1*Tb^2/2;
                pz_s = pz1 + v_z1*t - a_z1*Tb^2/2;
                sigma = [ px_s;
                          py_s;
                          pz_s];

        else
                px_s = px1 + (v_x1*T1 - a_x1*Tb^2) - a_x1*(T1 - t)^2 / 2 ;
                py_s = py1 + (v_y1*T1 - a_y1*Tb^2) - a_y1*(T1 - t)^2 / 2 ;
                pz_s = pz1 + (v_z1*T1 - a_z1*Tb^2) - a_z1*(T1 - t)^2 / 2 ;
                sigma = [ px_s;
                          py_s;
                          pz_s];
        end

        sigma_mag = sqrt( (sigma(1) - px1)^2 + (sigma(2)-py1)^2 + (sigma(3)-pz1)^2);
        orientation_sample = [ (nx1 + (nx2-nx1)*sigma_mag/L) (ox1 + (ox2-ox1)*sigma_mag/L) (ax1 + (ax2-ax1)*sigma_mag/L);
                               (ny1 + (ny2-ny1)*sigma_mag/L) (oy1 + (oy2-oy1)*sigma_mag/L) (ay1 + (ay2-ay1)*sigma_mag/L);
                               (nz1 + (nz2-nz1)*sigma_mag/L) (oz1 + (oz2-oz1)*sigma_mag/L) (az1 + (az2-az1)*sigma_mag/L)];

    sample = [sample, orientation_sample, sigma];
    end

result = sample;
end

%% Sampling Blended Trajectory
function result = Traj_Blend( position1, position2, position3, sample_num, v, a)
%% Obtain components of start and end position
nx1=  position1(1,1);
ny1=  position1(2,1);
nz1=  position1(3,1);
ox1=  position1(1,2) ;
oy1=  position1(2,2);
oz1=  position1(3,2);
ax1=  position1(1,3);
ay1=  position1(2,3);
az1=  position1(3,3);
px1=  position1(1,4);
py1=  position1(2,4);
pz1=  position1(3,4);

nx2=  position2(1,1);
ny2=  position2(2,1);
nz2=  position2(3,1);
ox2=  position2(1,2) ;
oy2=  position2(2,2);
oz2=  position2(3,2);
ax2=  position2(1,3);
ay2=  position2(2,3);
az2=  position2(3,3);
px2=  position2(1,4);
py2=  position2(2,4);
pz2=  position2(3,4);

nx3=  position3(1,1);
ny3=  position3(2,1);
nz3=  position3(3,1);
ox3=  position3(1,2) ;
oy3=  position3(2,2);
oz3=  position3(3,2);
ax3=  position3(1,3);
ay3=  position3(2,3);
az3=  position3(3,3);
px3=  position3(1,4);
py3=  position3(2,4);
pz3=  position3(3,4);

%% Obtain unit vector and length between two position
sample = [];
sample_time = [];
L1 = sqrt((px2 - px1)^2 + (py2 - py1)^2 + (pz2 - pz1)^2);
L2 = sqrt((px3 - px2)^2 + (py3 - py2)^2 + (pz3 - pz2)^2);
u1 = [(px2 - px1)/L1 (py2 - py1)/L1 (pz2 - pz1)/L1]';
u2 = [(px3 - px2)/L2 (py3 - py2)/L2 (pz3 - pz2)/L2]';
T1 = (L1*a + v^2)/(a*v);
T2 = (L2*a + v^2)/(a*v);
Tb = v/a;

a_x1 = a * u1(1);
a_y1 = a * u1(2);
a_z1 = a * u1(3);
v_x1 = v * u1(1);
v_y1 = v * u1(2);
v_z1 = v * u1(3);

a_x2 = a * u2(1);
a_y2 = a * u2(2);
a_z2 = a * u2(3);
v_x2 = v * u2(1);
v_y2 = v * u2(2);
v_z2 = v * u2(3);


% Sample
    for t = linspace(0, T1+T2-Tb, sample_num)
        sample_time = [sample_time, t];
        
        if t <= Tb
                px_s = px1 + (a_x1 * t^2) / 2;
                py_s = py1 + (a_y1 * t^2) / 2;
                pz_s = pz1 + (a_z1 * t^2) / 2;
                sigma = [ px_s;
                          py_s;
                          pz_s];
       
        elseif t > Tb && t < (T1 - Tb)
                px_s = px1 + v_x1*t - a_x1*Tb^2/2;
                py_s = py1 + v_y1*t - a_y1*Tb^2/2;
                pz_s = pz1 + v_z1*t - a_z1*Tb^2/2;
                sigma = [ px_s;
                          py_s;
                          pz_s];

        elseif t >= (T1 - Tb) && t <= T1 
                px_s = px1 + (v_x1*T1 - a_x1*Tb^2) - a_x1*(T1 - t)^2 / 2 + (a_x2*(t - (T1 - Tb))^2 / 2);
                py_s = py1 + (v_y1*T1 - a_y1*Tb^2) - a_y1*(T1 - t)^2 / 2 + (a_y2*(t - (T1 - Tb))^2 / 2);
                pz_s = pz1 + (v_z1*T1 - a_z1*Tb^2) - a_z1*(T1 - t)^2 / 2 + (a_z2*(t - (T1 - Tb))^2 / 2);
                sigma = [ px_s;
                          py_s;
                          pz_s];
                      
        elseif t > T1 && t < (T1 + T2 - 2*Tb)
                px_s = px2 + v_x2*(t - (T1 - Tb)) - a_x2*Tb^2/2;
                py_s = py2 + v_y2*(t - (T1 - Tb)) - a_y2*Tb^2/2;
                pz_s = pz2 + v_z2*(t - (T1 - Tb)) - a_z2*Tb^2/2;
                sigma = [ px_s;
                          py_s;
                          pz_s];
        else
                px_s = px2 + (v_x2*T2 - a_x2*Tb^2) - a_x2*((T1 + T2 - Tb) - t)^2 / 2;
                py_s = py2 + (v_y2*T2 - a_y2*Tb^2) - a_y2*((T1 + T2 - Tb) - t)^2 / 2;
                pz_s = pz2 + (v_z2*T2 - a_z2*Tb^2) - a_z2*((T1 + T2 - Tb) - t)^2 / 2;
                sigma = [ px_s;
                          py_s;
                          pz_s];
        end
        
        if t <= T1 - Tb
                
        sigma_mag = sqrt( (sigma(1) - px1)^2 + (sigma(2)-py1)^2 + (sigma(3)-pz1)^2);
        orientation_sample = [ (nx1 + (nx2-nx1)*sigma_mag/L1) (ox1 + (ox2-ox1)*sigma_mag/L1) (ax1 + (ax2-ax1)*sigma_mag/L1);
                               (ny1 + (ny2-ny1)*sigma_mag/L1) (oy1 + (oy2-oy1)*sigma_mag/L1) (ay1 + (ay2-ay1)*sigma_mag/L1);
                               (nz1 + (nz2-nz1)*sigma_mag/L1) (oz1 + (oz2-oz1)*sigma_mag/L1) (az1 + (az2-az1)*sigma_mag/L1)];

        sample = [sample, orientation_sample, sigma];
        
        elseif  t > (T1 - Tb) && t < (T1 + Tb)
            sigma_mag1 = sqrt( (sigma(1) - px1)^2 + (sigma(2)-py1)^2 + (sigma(3)-pz1)^2);
            sigma_mag2 = sqrt( (sigma(1) - px2)^2 + (sigma(2)-py2)^2 + (sigma(3)-pz2)^2);
             
            orientation_sample = [ (nx1 + (nx2-nx1)*sigma_mag/L1 + (nx3-nx2)*sigma_mag/L2), (ox1 + (ox2-ox1)*sigma_mag/L1 + (ox3-ox2)*sigma_mag/L2), (ax1 + (ax2-ax1)*sigma_mag/L1 + (ax3-ax2)*sigma_mag/L2);
                                   (ny1 + (ny2-ny1)*sigma_mag/L1 + (ny3-ny2)*sigma_mag/L2), (oy1 + (oy2-oy1)*sigma_mag/L1 + (oy3-oy2)*sigma_mag/L2), (ay1 + (ay2-ay1)*sigma_mag/L1 + (ay3-ay2)*sigma_mag/L2);
                                   (nz1 + (nz2-nz1)*sigma_mag/L1 + (nz3-nz2)*sigma_mag/L2), (oz1 + (oz2-oz1)*sigma_mag/L1 + (oz3-oz2)*sigma_mag/L2), (az1 + (az2-az1)*sigma_mag/L1 + (az3-az2)*sigma_mag/L2)];
            
            sample = [sample, orientation_sample, sigma];
        
        else
                
        sigma_mag = sqrt( (sigma(1) - px2)^2 + (sigma(2)-py2)^2 + (sigma(3)-pz2)^2);
        orientation_sample = [ (nx2 + (nx3-nx2)*sigma_mag/L2) (ox2 + (ox3-ox2)*sigma_mag/L2) (ax2 + (ax3-ax2)*sigma_mag/L2);
                               (ny2 + (ny3-ny2)*sigma_mag/L2) (oy2 + (oy3-oy2)*sigma_mag/L2) (ay2 + (ay3-ay2)*sigma_mag/L2);
                               (nz2 + (nz3-nz2)*sigma_mag/L2) (oz2 + (oz3-oz2)*sigma_mag/L2) (az2 + (az3-az2)*sigma_mag/L2)];

        sample = [sample, orientation_sample, sigma];
        end
    end

result = sample;
end

%% 4position Trajectory_Blending
function result = Traj_Blend_2( position1, position2, position3, position4, sample_num, v, a)
%% Obtain components of start and end position
nx1=  position1(1,1);
ny1=  position1(2,1);
nz1=  position1(3,1);
ox1=  position1(1,2) ;
oy1=  position1(2,2);
oz1=  position1(3,2);
ax1=  position1(1,3);
ay1=  position1(2,3);
az1=  position1(3,3);
px1=  position1(1,4);
py1=  position1(2,4);
pz1=  position1(3,4);

nx2=  position2(1,1);
ny2=  position2(2,1);
nz2=  position2(3,1);
ox2=  position2(1,2) ;
oy2=  position2(2,2);
oz2=  position2(3,2);
ax2=  position2(1,3);
ay2=  position2(2,3);
az2=  position2(3,3);
px2=  position2(1,4);
py2=  position2(2,4);
pz2=  position2(3,4);

nx3=  position3(1,1);
ny3=  position3(2,1);
nz3=  position3(3,1);
ox3=  position3(1,2) ;
oy3=  position3(2,2);
oz3=  position3(3,2);
ax3=  position3(1,3);
ay3=  position3(2,3);
az3=  position3(3,3);
px3=  position3(1,4);
py3=  position3(2,4);
pz3=  position3(3,4);

nx4=  position4(1,1);
ny4=  position4(2,1);
nz4=  position4(3,1);
ox4=  position4(1,2) ;
oy4=  position4(2,2);
oz4=  position4(3,2);
ax4=  position4(1,3);
ay4=  position4(2,3);
az4=  position4(3,3);
px4=  position4(1,4);
py4=  position4(2,4);
pz4=  position4(3,4);

%% Obtain unit vector and length between two position
sample = [];
sample_time = [];
L1 = sqrt((px2 - px1)^2 + (py2 - py1)^2 + (pz2 - pz1)^2);
L2 = sqrt((px3 - px2)^2 + (py3 - py2)^2 + (pz3 - pz2)^2);
L3 = sqrt((px4 - px3)^2 + (py4 - py3)^2 + (pz4 - pz3)^2);

u1 = [(px2 - px1)/L1 (py2 - py1)/L1 (pz2 - pz1)/L1]';
u2 = [(px3 - px2)/L2 (py3 - py2)/L2 (pz3 - pz2)/L2]';
u3 = [(px4 - px3)/L3 (py4 - py3)/L3 (pz4 - pz3)/L3]';

T1 = (L1*a + v^2)/(a*v);
T2 = (L2*a + v^2)/(a*v);
T3 = (L3*a + v^2)/(a*v);

Tb = v/a;

a_x1 = a * u1(1);
a_y1 = a * u1(2);
a_z1 = a * u1(3);
v_x1 = v * u1(1);
v_y1 = v * u1(2);
v_z1 = v * u1(3);

a_x2 = a * u2(1);
a_y2 = a * u2(2);
a_z2 = a * u2(3);
v_x2 = v * u2(1);
v_y2 = v * u2(2);
v_z2 = v * u2(3);

a_x3 = a * u3(1);
a_y3 = a * u3(2);
a_z3 = a * u3(3);
v_x3 = v * u3(1);
v_y3 = v * u3(2);
v_z3 = v * u3(3);


% Sample
    for t = linspace(0, T1+T2+T3 - 2*Tb, sample_num)
        sample_time = [sample_time, t];
        
        %% Position of EE
        if t <= Tb
                px_s = px1 + (a_x1 * t^2) / 2;
                py_s = py1 + (a_y1 * t^2) / 2;
                pz_s = pz1 + (a_z1 * t^2) / 2;
                sigma = [ px_s;
                          py_s;
                          pz_s];  
        elseif t > Tb && t < (T1 - Tb)
                px_s = px1 + v_x1*t - a_x1*Tb^2/2;
                py_s = py1 + v_y1*t - a_y1*Tb^2/2;
                pz_s = pz1 + v_z1*t - a_z1*Tb^2/2;
                sigma = [ px_s;
                          py_s;
                          pz_s];
        elseif t >= (T1 - Tb) && t <= T1 
                px_s = px1 + (v_x1*T1 - a_x1*Tb^2) - a_x1*(T1 - t)^2 / 2 + (a_x2*(t - (T1 - Tb))^2 / 2);
                py_s = py1 + (v_y1*T1 - a_y1*Tb^2) - a_y1*(T1 - t)^2 / 2 + (a_y2*(t - (T1 - Tb))^2 / 2);
                pz_s = pz1 + (v_z1*T1 - a_z1*Tb^2) - a_z1*(T1 - t)^2 / 2 + (a_z2*(t - (T1 - Tb))^2 / 2);
                sigma = [ px_s;
                          py_s;
                          pz_s];                   
        elseif t > T1 && t < (T1 + T2 - 2*Tb)
                px_s = px2 + v_x2*(t - (T1 - Tb)) - a_x2*Tb^2/2;
                py_s = py2 + v_y2*(t - (T1 - Tb)) - a_y2*Tb^2/2;
                pz_s = pz2 + v_z2*(t - (T1 - Tb)) - a_z2*Tb^2/2;
                sigma = [ px_s;
                          py_s;
                          pz_s];
        elseif t >= (T1 + T2 - 2*Tb) && t <= (T1 + T2 - Tb)
                px_s = px2 + (v_x2*T2 - a_x2*Tb^2) - a_x2*((T1 + T2 - Tb) - t)^2 / 2 + (a_x3*(t - (T1+T2 - 2*Tb))^2 / 2);
                py_s = py2 + (v_y2*T2 - a_y2*Tb^2) - a_y2*((T1 + T2 - Tb) - t)^2 / 2 + (a_y3*(t - (T1+T2 - 2*Tb))^2 / 2);
                pz_s = pz2 + (v_z2*T2 - a_z2*Tb^2) - a_z2*((T1 + T2 - Tb) - t)^2 / 2 + (a_z3*(t - (T1+T2 - 2*Tb))^2 / 2);
                sigma = [ px_s;
                          py_s;
                          pz_s];
        elseif t > (T1 + T2 - Tb) && t < (T1 + T2 + T3 - 3*Tb)
                px_s = px3 + v_x3*(t - (T1+T2 - 2*Tb)) - a_x3*Tb^2/2;
                py_s = py3 + v_y3*(t - (T1+T2 - 2*Tb)) - a_y3*Tb^2/2;
                pz_s = pz3 + v_z3*(t - (T1+T2 - 2*Tb)) - a_z3*Tb^2/2;
                sigma = [ px_s;
                          py_s;
                          pz_s];            
        else
            px_s = px3 + (v_x3*T3 - a_x3*Tb^2) - a_x3*((T1+T2+T3 - 2*Tb) - t)^2 / 2;
            py_s = py3 + (v_y3*T3 - a_y3*Tb^2) - a_y3*((T1+T2+T3 - 2*Tb) - t)^2 / 2;
            pz_s = pz3 + (v_z3*T3 - a_z3*Tb^2) - a_z3*((T1+T2+T3 - 2*Tb) - t)^2 / 2;
            sigma = [ px_s;
                      py_s;
                      pz_s];
        end
        
       %% Orientation of EE (Rotation Matrix)
        if t <= T1 - Tb
        sigma_mag = sqrt( (sigma(1) - px1)^2 + (sigma(2)-py1)^2 + (sigma(3)-pz1)^2);
        orientation_sample = [ (nx1 + (nx2-nx1)*sigma_mag/L1) (ox1 + (ox2-ox1)*sigma_mag/L1) (ax1 + (ax2-ax1)*sigma_mag/L1);
                               (ny1 + (ny2-ny1)*sigma_mag/L1) (oy1 + (oy2-oy1)*sigma_mag/L1) (ay1 + (ay2-ay1)*sigma_mag/L1);
                               (nz1 + (nz2-nz1)*sigma_mag/L1) (oz1 + (oz2-oz1)*sigma_mag/L1) (az1 + (az2-az1)*sigma_mag/L1)];

        sample = [sample, orientation_sample, sigma];
        
        elseif  t >= (T1 - Tb) && t <= T1
            sigma_mag1 = sqrt( (sigma(1) - px1)^2 + (sigma(2)-py1)^2 + (sigma(3)-pz1)^2);
            sigma_mag2 = sqrt( (sigma(1) - px2)^2 + (sigma(2)-py2)^2 + (sigma(3)-pz2)^2);
             
            orientation_sample = [ (nx1 + (nx2-nx1)*sigma_mag1/L1 + (nx3-nx2)*sigma_mag2/L2), (ox1 + (ox2-ox1)*sigma_mag1/L1 + (ox3-ox2)*sigma_mag2/L2), (ax1 + (ax2-ax1)*sigma_mag1/L1 + (ax3-ax2)*sigma_mag2/L2);
                                   (ny1 + (ny2-ny1)*sigma_mag1/L1 + (ny3-ny2)*sigma_mag2/L2), (oy1 + (oy2-oy1)*sigma_mag1/L1 + (oy3-oy2)*sigma_mag2/L2), (ay1 + (ay2-ay1)*sigma_mag1/L1 + (ay3-ay2)*sigma_mag2/L2);
                                   (nz1 + (nz2-nz1)*sigma_mag1/L1 + (nz3-nz2)*sigma_mag2/L2), (oz1 + (oz2-oz1)*sigma_mag1/L1 + (oz3-oz2)*sigma_mag2/L2), (az1 + (az2-az1)*sigma_mag1/L1 + (az3-az2)*sigma_mag2/L2)];
            
            sample = [sample, orientation_sample, sigma];
        
        elseif t > T1 && t < T1+T2 - 2*Tb
                
        sigma_mag = sqrt( (sigma(1) - px2)^2 + (sigma(2)-py2)^2 + (sigma(3)-pz2)^2);
        orientation_sample = [ (nx2 + (nx3-nx2)*sigma_mag/L2) (ox2 + (ox3-ox2)*sigma_mag/L2) (ax2 + (ax3-ax2)*sigma_mag/L2);
                               (ny2 + (ny3-ny2)*sigma_mag/L2) (oy2 + (oy3-oy2)*sigma_mag/L2) (ay2 + (ay3-ay2)*sigma_mag/L2);
                               (nz2 + (nz3-nz2)*sigma_mag/L2) (oz2 + (oz3-oz2)*sigma_mag/L2) (az2 + (az3-az2)*sigma_mag/L2)];

        sample = [sample, orientation_sample, sigma];
        
        elseif t >= (T1+T2 - 2*Tb) && t <= (T1+T2 - Tb)
            sigma_mag2 = sqrt( (sigma(1) - px2)^2 + (sigma(2)-py2)^2 + (sigma(3)-pz2)^2);
            sigma_mag3 = sqrt( (sigma(1) - px3)^2 + (sigma(2)-py3)^2 + (sigma(3)-pz3)^2);
           
            orientation_sample = [ (nx2 + (nx3-nx2)*sigma_mag2/L2 + (nx4-nx3)*sigma_mag3/L3), (ox2 + (ox3-ox2)*sigma_mag2/L2 + (ox4-ox3)*sigma_mag3/L3), (ax2 + (ax3-ax2)*sigma_mag2/L2 + (ax4-ax3)*sigma_mag3/L3);
                                   (ny2 + (ny3-ny2)*sigma_mag2/L2 + (ny4-ny3)*sigma_mag3/L3), (oy2 + (oy3-oy2)*sigma_mag2/L2 + (oy4-oy3)*sigma_mag3/L3), (ay2 + (ay3-ay2)*sigma_mag2/L2 + (ay4-ay3)*sigma_mag3/L3);
                                   (nz2 + (nz3-nz2)*sigma_mag2/L2 + (nz4-nz3)*sigma_mag3/L3), (oz2 + (oz3-oz2)*sigma_mag2/L2 + (oz4-oz3)*sigma_mag3/L3), (az2 + (az3-az2)*sigma_mag2/L2 + (az4-az3)*sigma_mag3/L3)];
            
            sample = [sample, orientation_sample, sigma];
            
        else
            sigma_mag = sqrt( (sigma(1) - px3)^2 + (sigma(2)-py3)^2 + (sigma(3)-pz3)^2);
            orientation_sample = [ (nx3 + (nx4-nx3)*sigma_mag/L3) (ox3 + (ox4-ox3)*sigma_mag/L3) (ax3 + (ax4-ax3)*sigma_mag/L3);
                                   (ny3 + (ny4-ny3)*sigma_mag/L3) (oy3 + (oy4-oy3)*sigma_mag/L3) (ay3 + (ay4-ay3)*sigma_mag/L3);
                                   (nz3 + (nz4-nz3)*sigma_mag/L3) (oz3 + (oz4-oz3)*sigma_mag/L3) (az3 + (az4-az3)*sigma_mag/L3)];

        sample = [sample, orientation_sample, sigma];
        end
    end

result = sample;
end


%% Inverse Kinematics
function result = IK( EE_position )
% length
h  = 0.063 ;
l1 = 0.153 ;
l2 = 0.153 ;
l4 = 0.098 ;

%% element
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

% EE_array  = 
%
%[nx ox ax px;
% ny oy ay py;
% nz oz az pz;
% 0  0  0  1 ];
 
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
 
 q_array = [i1 i2 i3 i4];
 result = q_array;
 
end




%% Function for r2d
function result = r2d(radian)
    result = radian*180/pi;
end
