%% DH convention for Task A1    
%  Taegyun Ha

%% Cear windows
close all
clc
disp('The following code simulates the FK')

%% Links Lengths
org = [0, 0, 0, 1]';
d1 = 0.063 ;
a2 = 0.153 ;
a3 = 0.153 ;
de = 0.068 ;

%% q1, q2, q3, q4 radians
q_ar = [0 2.4592 -2.5877 0.6521]';

q1 =  q_ar(1); %[ 90  90];
q2 =  q_ar(2); %[ 90  40];
q3 =  q_ar(3); %[-90 -80];
q4 =  q_ar(4); %[ 90  40];

%% Derive Transformation matrix using Distal and Proximal
T01 = distal(0    , pi/2, d1   , q1);
T12 = distal(a2   , 0   , 0    , q2);
T23 = distal(a3   , 0   , 0    , q3);
T34 = distal(0    , pi/2, 0    , q4);
T45 = distal(0    , 0   , de   , 0        );
T05 = T01*T12*T23*T34*T45;
je = T01*T12*T23*T34*T45 * org;

T01p = prox(0    , 0   , d1   , q1);
T12p = prox(a2   , 0   , 0    , q2);
T23p = prox(a3   , 0   , 0    , q3);
T34p = prox(0    , pi/2, 0    , q4);
T45p = prox(0    , 0   , de   , 0        );
T5ep = prox(0    , 0   , de   , 0        );

T05p = T01*T12*T23*T34*T45;
je = T01*T12*T23*T34*T45 * org;

%% Function for Distal and Proximal
function result = distal( a_n, alph_n, d_n, th_n)

result = [      cos(th_n)        -cos(alph_n)*sin(th_n)    sin(alph_n)*sin(th_n)  a_n*cos(th_n);
                sin(th_n)         cos(alph_n)*cos(th_n)   -sin(alph_n)*cos(th_n)  a_n*sin(th_n);
                  0                   sin(alph_n)               cos(alph_n)            d_n;
                  0                       0                         0                   1      ];
end

function result = prox( a_nm1, alph_nm1, d_n, th_n)

result = [      cos(th_n)              -sin(th_n)                0              a_nm1;
          sin(th_n)*cos(alph_nm1) cos(th_n)*cos(alph_nm1) -sin(alph_nm1) -sin(alph_nm1)*d_n;
          sin(th_n)*sin(alph_nm1) cos(th_n)*sin(alph_nm1)  cos(alph_nm1)  cos(alph_nm1)*d_n;
                  0                       0                       0               1           ];
end
