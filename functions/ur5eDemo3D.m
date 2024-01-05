function [T, JointPos] = ur5eDemo3D(angle, Radians, Endeffector, EndtrjOpt, CoordOpt, t)
% angle: angle of each joint
% R: collision radians
% Endeffector: end effector position information
% EndtrjOpt: show end effector trajectory -> 1: turn on; 0: turn off 
% CoordOpt: show tf -> 1: turn on; 0: turn off
%% parameters
% UR5e dh parameters + 2 joints
a = [0, 0, -0.425, 0, -0.3922, 0, 0, 0]*1000;
d = [0.1625, 0.1333, 0, -0.1333, 0, 0.1333, 0.0997, 0.0996]*1000;
alpha = [pi/2, 0, 0, 0, 0, pi/2, -pi/2, 0];
theta = [angle(1:2) 0 angle(3) 0 angle(4:6)];
% forward kinematics
[T, JointPos] = fkineUR5e(theta, d, a, alpha);
% axis lengths
axisL = 60;
% base size
baseL = 100;
BaseCorner = [-baseL baseL 0; baseL baseL 0; baseL -baseL 0; -baseL -baseL 0; -baseL baseL 0];
%% Draw joints/coordinates
for i = 1:length(JointPos)
    % Plot joint coordinate
    plot3(JointPos(i, 1),JointPos(i, 2),JointPos(i, 3),'bo','MarkerFaceColor','b','MarkerSize',3);hold on
    if CoordOpt == 1
        Color = {'r','g','b'}; % Color for 3 axis
        % Draw joint coordinate
        EndPosx =  T{i}*[axisL; 0; 0; 1]; EndPosx = EndPosx(1:3)';% xaxis endpoint
        EndPosy =  T{i}*[0; axisL; 0; 1]; EndPosy = EndPosy(1:3)';% yaxis endpoint
        EndPosz =  T{i}*[0; 0; axisL; 1]; EndPosz = EndPosz(1:3)';% zaxis endpoint
        EndPos = [EndPosx; EndPosy; EndPosz];
        for j = 1: 3
            plot3([JointPos(i, 1),EndPos(j, 1)],[JointPos(i, 2),EndPos(j, 2)],[JointPos(i, 3),EndPos(j, 3)], Color{j}, 'LineWidth', 1);hold on
        end
    end
end
%% Draw links
for i = 1: length(JointPos) - 1
    plot3([JointPos(i,1),JointPos(i + 1,1)],[JointPos(i,2),JointPos(i + 1,2)],[JointPos(i,3),JointPos(i + 1,3)],'k','LineWidth',1);hold on
end
%% Draw base
plot3(BaseCorner(:,1),BaseCorner(:,2),BaseCorner(:,3),'k-'); hold on;
fill3(BaseCorner(:,1),BaseCorner(:,2),BaseCorner(:,3),'b'); hold on;
%% Draw Collision Capsule
for i = 1: length(JointPos) - 2
    LinkRadians = ones(length(JointPos) - 2, 1)*Radians;
    Cap1 = [JointPos(i,1),JointPos(i,2),JointPos(i,3)];
    Cap2 = [JointPos(i+1,1),JointPos(i+1,2),JointPos(i+1,3)];
    DrawCapsule(Cap1, Cap2, LinkRadians(i)); hold on
end
%% Draw Endeffector trajectory
if EndtrjOpt == 1
    plot3(Endeffector(:, 1), Endeffector(:, 2), Endeffector(:, 3), '.k', 'LineWidth', 1); hold on;
end
%% Adjust other settings
title(['simulation time = ', num2str(t), ' s']);
% light
light('Position',[0 -1500 1500],'Style','infinite');
light('Position',[1500 0 1500],'Style','infinite');
% axis range
axis equal;
axis([-1000, 1000,-1000, 1000, -100, 1000]);
set(gcf,'color','w');
% set(gca,'FontSize',5);
box on;
xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)');
view([60, 30]); drawnow;
hold off;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Subfunctions
function DrawCapsule(Pos1,Pos2, R)
% Joint Info
x1 = Pos1(1);
y1 = Pos1(2);
z1 = Pos1(3);

x2 = Pos2(1);
y2 = Pos2(2);
z2 = Pos2(3);

V1 = [x1,y1,z1];
V2 = [x2,y2,z2];

% First sphere
[Xs,Ys,Zs] = sphere(20);
Xs1 = Xs*(R+10) + V1(1);
Ys1 = Ys*(R+10) + V1(2);
Zs1 = Zs*(R+10) + V1(3);

% Second sphere
Xs2 = Xs*(R+10) + V2(1);
Ys2 = Ys*(R+10) + V2(2);
Zs2 = Zs*(R+10) + V2(3);

% Plot
% surf(Xs1,Ys1,Zs1,'EdgeColor','none','FaceColor','r','FaceAlpha',0.7); hold on
surf(Xs2,Ys2,Zs2,'EdgeColor','none','FaceColor',[0.3010 0.7450 0.9330],'FaceAlpha',0.7); hold on
[Xc,Yc,Zc]=cylinder2P(R,20,V1,V2);
surf(Xc,Yc,Zc,'EdgeColor','none','FaceColor',[0.3010 0.7450 0.9330],'FaceAlpha',0.7);  hold on

end

%  CYLINDER:  A function to draw a N-sided cylinder based on the
%             generator curve in the vector R.
%
%  Usage:      [X, Y, Z] = cylinder(R, N)
%
%  Arguments:  R - The vector of radii used to define the radius of
%                  the different segments of the cylinder.
%              N - The number of points around the circumference.
%
%  Returns:    X - The x-coordinates of each facet in the cylinder.
%              Y - The y-coordinates of each facet in the cylinder.
%              Z - The z-coordinates of each facet in the cylinder.
%
%  Author:     Luigi Barone
%  Date:       9 September 2001
%  Modified:   Per Sundqvist July 2004

function [X, Y, Z] = cylinder2P(R, N, r1, r2)

% The parametric surface will consist of a series of N-sided
% polygons with successive radii given by the array R.
% Z increases in equal sized steps from 0 to 1.

% Set up an array of angles for the polygon.
theta = linspace(0, 2*pi, N);

m = length(R);                 % Number of radius values
                           % supplied.

if m == 1                      % Only one radius value supplied.
    R = [R; R];                % Add a duplicate radius to make
    m = 2;                     % a cylinder.
end


X = zeros(m, N);             % Preallocate memory.
Y = zeros(m, N);
Z = zeros(m, N);

v=(r2-r1)/sqrt((r2-r1)*(r2-r1)');    %Normalized vector;
%cylinder axis described by: r(t)=r1+v*t for 0<t<1
R2=rand(1,3);              %linear independent vector (of v)
x2=v-R2/(R2*v');    %orthogonal vector to v
x2=x2/sqrt(x2*x2');     %orthonormal vector to v
x3=cross(v,x2);     %vector orthonormal to v and x2
x3=x3/sqrt(x3*x3');

r1x=r1(1);r1y=r1(2);r1z=r1(3);
r2x=r2(1);r2y=r2(2);r2z=r2(3);
vx=v(1);vy=v(2);vz=v(3);
x2x=x2(1);x2y=x2(2);x2z=x2(3);
x3x=x3(1);x3y=x3(2);x3z=x3(3);

time=linspace(0,1,m);
for j = 1 : m
    t=time(j);
    X(j, :) = r1x+(r2x-r1x)*t+R(j)*cos(theta)*x2x+R(j)*sin(theta)*x3x; 
    Y(j, :) = r1y+(r2y-r1y)*t+R(j)*cos(theta)*x2y+R(j)*sin(theta)*x3y; 
    Z(j, :) = r1z+(r2z-r1z)*t+R(j)*cos(theta)*x2z+R(j)*sin(theta)*x3z;
end
end
