%% Workspace of Parallel robot for task 2-2
% Taegyun Ha

%% Cear windows
close all
clc

%% Platform position input
alpha = 25;

%% Origin position of base, vector BC
pb = [ 0 0 0 ]';

%% Array for Workspace
wsx = [];
wsy = [];

%% Length
ra = 170;
L  = 130;
rp = 130;
rb = 290;

%% Base position
bpb1 = [-cos(d2r(30)) * rb;
        -sin(d2r(30)) * rb;
                0        ];
            
bpb2 = [ cos(d2r(30)) * rb;
        -sin(d2r(30)) * rb;
                0        ];
            
bpb3 = [        0         ;
                rb        ;
                0        ];

%% Vector for xpp, ypp
cpp1 = [-cos(d2r(30)) * rp;
        -sin(d2r(30)) * rp;
                 0       ];
             
cpp2 = [ cos(d2r(30)) * rp;
        -sin(d2r(30)) * rp;
                 0       ];
             
cpp3 = [         0        ;
                 rp       ;
                 0       ];

%% Rotation Matrix for Vector cpp
rbc = [cos(d2r(alpha)) -sin(d2r(alpha))   0;
       sin(d2r(alpha))  cos(d2r(alpha))   0;
           0                0             1];

%% Calculation for Workspace
for cx = bpb1(1) : 1 : bpb2(1)
    for cy = bpb1(2) : 1 : bpb3(2)    
        pc = [ cx cy 0]';
        bc = pc - pb;
        
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

       if (sqrt(xpp1^2 + ypp1^2) < (ra + L)) && (sqrt(xpp2^2 + ypp2^2) < (ra + L)) && (sqrt(xpp3^2 + ypp3^2) < (ra + L))
           if  ( sqrt( xpp1^2 + ypp1^2 ) > ra - L ) && ( sqrt( xpp2^2 + ypp2^2 ) > ra - L ) && ( sqrt( xpp3^2 + ypp3^2 ) > ra - L )
                wsx = [wsx, cx];      
                wsy = [wsy, cy];
           end
        end
    end
end

%% plot
figure (1) 
set(1,'position',[116 190 560 420])
plot( [bpb1(1) bpb2(1) bpb3(1) bpb1(1)], [bpb1(2) bpb2(2) bpb3(2) bpb1(2)], 'o-b');
grid;
xlabel('x (mm)');
ylabel('y (mm)');
title(strcat('Workspace alpha(deg)=',string(alpha)));
hold on
axis equal

viscircles([bpb1(1) bpb1(2)], ra+L,'LineWidth', 0.1 );
viscircles([bpb2(1) bpb2(2)], ra+L,'LineWidth', 0.1);
viscircles([bpb3(1) bpb3(2)], ra+L,'LineWidth', 0.1);

% debug
viscircles([bpb1(1) bpb1(2)], ra - L , 'LineWidth', 0.1);

plot( wsx, wsy, '.');

%% Function for d2r
function result = d2r(degree)
    result = degree*pi/180;
end

%% Function for r2d
function result = r2d(radian)
    result = radian*180/pi;
end
