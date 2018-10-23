%% Workspace of the centre of the wrist for Task A3
% Taegyun Ha

%% Cear windows
close all
clc

%% Links Lengths
org = [0, 0, 0, 1]';
d1 = 0.063 ;
a2 = 0.153 ;
a3 = 0.153 ;
de = 0.068 ;

%% Minimum, Maximum angle value - Degrees
q1min =    0;
q1max =  180;
q2min =    0;
q2max =  140;
q3min = -150;
q3max =    0;
q4min =  -180;
q4max =  10;
                

%% Plot the workspace of the robot
figure (1)
set(1,'position',[560 190 560 420]) % Set a window position

xwork = zeros(360,360) ; % reserving space for the variables, because
ywork = zeros(360,360) ; % otherwise they would be created later within a loop.

for q1 = q1min:20:q1max
    for q2 = q2min:20:q2max
        for q3 = q3min:20:q3max
            for q4 = q4min:20:q4max
                
                T01 = distal(0    , pi/2, d1   , q1*pi/180);
                T12 = distal(a2   , 0   , 0    , q2*pi/180);
                T23 = distal(a3   , 0   , 0    , q3*pi/180);
                T34 = distal(0    , pi/2, 0    , q4*pi/180);
                T45 = distal(0    , 0   , de   , 0        );
 
                je = T01*T12*T23*T34*T45 * org;
               
                xlabel('x (m)') ; ylabel('y (m)') ; zlabel('z (m)')
              
                
                plot3(je(1), je(2), je(3) ,'.b')
                hold on
                axis equal
            end
        end
    end
end

%% Function for Proximal

function result = prox( a_nm1, alph_nm1, d_n, th_n)

result = [      cos(th_n)              -sin(th_n)                0              a_nm1;
          sin(th_n)*cos(alph_nm1) cos(th_n)*cos(alph_nm1) -sin(alph_nm1) -sin(alph_nm1)*d_n;
          sin(th_n)*sin(alph_nm1) cos(th_n)*sin(alph_nm1)  cos(alph_nm1)  cos(alph_nm1)*d_n;
                  0                       0                       0               1           ];
end

%% Function for Distal

function result = distal( a_n, alph_n, d_n, th_n)

result = [      cos(th_n)        -cos(alph_n)*sin(th_n)    sin(alph_n)*sin(th_n)  a_n*cos(th_n);
                sin(th_n)         cos(alph_n)*cos(th_n)   -sin(alph_n)*cos(th_n)  a_n*sin(th_n);
                  0                   sin(alph_n)               cos(alph_n)            d_n;
                  0                       0                         0                   1      ];
end
