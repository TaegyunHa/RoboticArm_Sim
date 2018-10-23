%% Inverse Kinematics of Parallel robot for task 2-1
% Taegyun Ha

%% Cear windows
close all
clc

%% Platform position input
alpha = -20;
pc = [ 10 30 0 ]';

%% Origin position of base, vector BC
pb = [ 0 0 0 ]';
bc = pc - pb;
% xpp = ra*cos(the) + L*cos(psi);
% ypp = ra*sin(the) + L*sin(psi);
 
%% Length
ra = 170;
L  = 130;
rp = 130;
rb = 290;

%% Vector for xpp, ypp
rbc = [cos(d2r(alpha)) -sin(d2r(alpha))   0;
       sin(d2r(alpha))  cos(d2r(alpha))   0;
           0                0             1];
cpp1 = [-cos(d2r(30)) * rp;
        -sin(d2r(30)) * rp;
                 0       ];
             
cpp2 = [ cos(d2r(30)) * rp;
        -sin(d2r(30)) * rp;
                 0       ];
             
cpp3 = [         0        ;
                 rp       ;
                 0       ];
             
bpb1 = [-cos(d2r(30)) * rb;
        -sin(d2r(30)) * rb;
                0        ];
bpb2 = [ cos(d2r(30)) * rb;
        -sin(d2r(30)) * rb;
                0        ];
bpb3 = [        0         ;
                rb        ;
                0        ];
   
bpp1 = rbc*cpp1 + bc;
pbpp1 = bpp1 - bpb1;
xpp1 = pbpp1(1);
ypp1 = pbpp1(2);

bpp2 = rbc*cpp2 + bc;
pbpp2 = bpp2 - bpb2;
xpp2 = pbpp2(1);
ypp2 = pbpp2(2);

bpp3 = rbc*cpp3 + bc;
pbpp3 = bpp3 - bpb3;
xpp3 = pbpp3(1);
ypp3 = pbpp3(2);

%% equation for pb3
e11 = -2*ypp1*ra;
e12 = -2*xpp1*ra;
e13 = xpp1^2 + ypp1^2 + ra^2 - L^2;

t11 = (-e11 + sqrt(e11^2 + e12^2 - e13^2))/(e13-e12);
t12 = (-e11 - sqrt(e11^2 + e12^2 - e13^2))/(e13-e12);
theta11 = 2*atan(t11);
theta12 = 2*atan(t12);

e21 = -2*ypp2*ra;
e22 = -2*xpp2*ra;
e23 = xpp2^2 + ypp2^2 + ra^2 - L^2;

t21 = (-e21 + sqrt(e21^2 + e22^2 - e23^2))/(e23-e22);
t22 = (-e21 - sqrt(e21^2 + e22^2 - e23^2))/(e23-e22);
theta21 = 2*atan(t21);
theta22 = 2*atan(t22);

e31 = -2*ypp3*ra;
e32 = -2*xpp3*ra;
e33 = xpp3^2 + ypp3^2 + ra^2 - L^2;

t31 = (-e31 + sqrt(e31^2 + e32^2 - e33^2))/(e33-e32);
t32 = (-e31 - sqrt(e31^2 + e32^2 - e33^2))/(e33-e32);
theta31 = 2*atan(t31);
theta32 = 2*atan(t32);

%% Position M
pbm11 = bpb1 + [ra*cos(theta11) , ra*sin(theta11), 0]';
pbm21 = bpb2 + [ra*cos(theta21) , ra*sin(theta21), 0]';
pbm31 = bpb3 + [ra*cos(theta31) , ra*sin(theta31), 0]';

pbm12 = bpb1 + [ra*cos(theta12) , ra*sin(theta12), 0]';
pbm22 = bpb2 + [ra*cos(theta22) , ra*sin(theta22), 0]';
pbm32 = bpb3 + [ra*cos(theta32) , ra*sin(theta32), 0]';

%% plot
figure (1) 
set(1,'position',[116 190 560 420])
plot( [bpb1(1) bpb2(1) bpb3(1) bpb1(1)], [bpb1(2) bpb2(2) bpb3(2) bpb1(2)], 'b');
grid;
xlabel('x (mm)');
ylabel('y (mm)');
title(strcat('Cartesian position X:', string(pc(1)),'(mm), Y:', string(pc(2)),'(mm), alpha(deg)=',string(alpha)));
hold on
axis equal

%plot( [ cpp1(1) cpp2(1) cpp3(1) cpp1(1) ], [ cpp1(2) cpp2(2) cpp3(2) cpp1(2) ], 'b');
plot( [ bpp1(1) bpp2(1) bpp3(1) bpp1(1) ], [ bpp1(2) bpp2(2) bpp3(2) bpp1(2) ], '-or');
plot( pc(1), pc(2), '.r');

plot( [bpb1(1) pbm11(1)], [bpb1(2) pbm11(2)], 'g');
plot( [pbm11(1) bpp1(1)], [pbm11(2) bpp1(2)], 'b');
plot( [bpb2(1) pbm21(1)], [bpb2(2) pbm21(2)], 'g');
plot( [pbm21(1) bpp2(1)], [pbm21(2) bpp2(2)], 'b');
plot( [bpb3(1) pbm31(1)], [bpb3(2) pbm31(2)], 'g');
plot( [pbm31(1) bpp3(1)], [pbm31(2) bpp3(2)], 'b');

plot( [bpb1(1) pbm12(1)], [bpb1(2) pbm12(2)], 'y');
plot( [pbm12(1) bpp1(1)], [pbm12(2) bpp1(2)], 'y');
plot( [bpb2(1) pbm22(1)], [bpb2(2) pbm22(2)], 'y');
plot( [pbm22(1) bpp2(1)], [pbm22(2) bpp2(2)], 'y');
plot( [bpb3(1) pbm32(1)], [bpb3(2) pbm32(2)], 'y');
plot( [pbm32(1) bpp3(1)], [pbm32(2) bpp3(2)], 'y');

str1 = strcat('еш1=', string(theta11*180/pi),'вк');
str2 = strcat('еш2=', string(theta21*180/pi),'вк');
str3 = strcat('еш3=', string(theta31*180/pi),'вк');
text( bpb1(1)+30, bpb1(2)+20, str1);
text( bpb2(1)-140, bpb2(2)+20, str2);
text( bpb3(1)+30, bpb3(2)-20, str3);

%% Function for d2r
function result = d2r(degree)
    result = degree*pi/180;
end

%% Function for r2d
function result = r2d(radian)
    result = radian*180/pi;
end
