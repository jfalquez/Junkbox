
% This function generates a sequence of 6d poses defining a trajectory
% around the simulated CityBlock dataset. The 6d poses can be computed
% in absolute or relative coordinates. The CityBlock world has it's origin
% at (0,0), interior walls at 5m and exterior walls at 10m.
% 
% Input:
%     coord   (0) if absolute (1) if relative. [default = 0]
%     fstep   incremental forward motion of the robot [default = 0.20m] 
%     zstep   incremental height change [default = 0];
%     rad     radius that defines the turning angle at the corners
%             [default = 1.0m]
%     nlaps   number of laps [default = 1];

function T = generateTrajectory(coord,fstep,zstep,rad,nlaps)

%****************************************************
% Validate args
%****************************************************
if(nargin < 1), coord = 0;   end
if(nargin < 2), fstep = 0.2; end
if(nargin < 3), zstep = 0.0; end
if(nargin < 4), rad   = 1.0; end
if(nargin < 5), nlaps = 1;   end

%****************************************************
% Compute trajectory
%****************************************************

spnt = [-1.0 0 1];

slim_up    = 1.5-rad;
slim_down  = -1.5+rad;
slim_left  = 1.0-rad;
slim_right = -1.0+rad;

T = [spnt 0 0 pi]; % [x y z roll pitch yaw]

for k=1:nlaps
    
    % left straight
    fixed_coord = T(end,1);
    nsteps = (slim_up - T(end,2))/fstep;
    for i=1:nsteps
        T = [T; fixed_coord T(end,2)+fstep T(end,3)+zstep 0 0 pi];
    end
    
    % top-left turn
    if(rad > 0)
        cx = slim_left;
        cy = slim_up;

        for ang=pi:-0.05:0.5*pi
            T = [T; rad*cos(ang) + cx rad*sin(ang)+cy T(end,3)+zstep 0 0 ang];
        end
    end

    % top straight
    nsteps = (slim_right - T(end,1))/fstep;
    fixed_coord = T(end,2);
    for i=1:nsteps
        T = [T; T(end,1)+fstep fixed_coord T(end,3)+zstep 0 0 0.5*pi];
    end
    
    % top-right turn
    if(rad > 0)
        cx = slim_right;
        cy = slim_up;

        for ang=0.5*pi:-0.05:0
            T = [T; rad*cos(ang) + cx rad*sin(ang)+cy T(end,3)+zstep 0 0 ang];
        end
    end

    % right straight
    nsteps = abs(slim_down - T(end,2))/fstep;
    fixed_coord = T(end,1);
    for i=1:nsteps
        T = [T; fixed_coord T(end,2)-fstep T(end,3)+zstep 0 0 0];
    end
    
    % bottom-right turn
    if(rad > 0)
        cx = slim_right;
        cy = slim_down;

        for ang=0:-0.05:-0.5*pi
            T = [T; rad*cos(ang) + cx rad*sin(ang)+cy T(end,3)+zstep 0 0 ang];
        end
    end
    
    % bottom straight
    nsteps = abs(slim_left - T(end,1))/fstep;
    fixed_coord = T(end,2);
    for i=1:nsteps
        T = [T; T(end,1)-fstep fixed_coord T(end,3)+zstep 0 0 -0.5*pi];
    end
    
    % bottom-left turn
    if(rad > 0)
        cx = slim_left;
        cy = slim_down;

        for ang=-0.5*pi:-0.05:-pi
            T = [T; rad*cos(ang) + cx rad*sin(ang)+cy T(end,3)+zstep 0 0 ang];
        end
    end
    
    % close loop (left)
    nsteps = (spnt(2) - T(end,2))/fstep; 
    fixed_coord = T(end,1);
    for i=1:nsteps
        T = [T; fixed_coord T(end,2)+fstep T(end,3)+zstep 0 0 pi];
    end

end

%****************************************************
% Draw simulated trajectory
%****************************************************

hold on;

%draw trajectory
plot3DCurveColormap(T);
%plot3(T(:,1),T(:,2),T(:,3),'-r');

%Draw inner walls
H = 10;
D = 5;
top = [-D D H; D D H; D -D H; -D -D H; -D D H];
bot = [-D D 0; D D 0; D -D 0; -D -D 0; -D D 0];
plot3(top(:,1),top(:,2),top(:,3),'-k');
plot3(bot(:,1),bot(:,2),bot(:,3),'-k');
line = [-D  D 0; -D  D H]; plot3(line(:,1),line(:,2),line(:,3),'-k');
line = [ D -D 0;  D -D H]; plot3(line(:,1),line(:,2),line(:,3),'-k');
line = [ D  D 0;  D  D H]; plot3(line(:,1),line(:,2),line(:,3),'-k');
line = [-D -D 0; -D -D H]; plot3(line(:,1),line(:,2),line(:,3),'-k');


%Draw outer walls
D = 10;
top = [-D D H; D D H; D -D H; -D -D H; -D D H];
bot = [-D D 0; D D 0; D -D 0; -D -D 0; -D D 0];
plot3(top(:,1),top(:,2),top(:,3),'-k');
plot3(bot(:,1),bot(:,2),bot(:,3),'-k');
line = [-D  D 0; -D  D H]; plot3(line(:,1),line(:,2),line(:,3),'-k');
line = [ D -D 0;  D -D H]; plot3(line(:,1),line(:,2),line(:,3),'-k');
line = [ D  D 0;  D  D H]; plot3(line(:,1),line(:,2),line(:,3),'-k');
line = [-D -D 0; -D -D H]; plot3(line(:,1),line(:,2),line(:,3),'-k');

%****************************************************
% Convert to robotics coord
%****************************************************

npoints = size(T,1);
T = [T(:,2) T(:,1) -T(:,3) T(:,[4 5]) pi*ones(npoints,1)-T(:,6)];


return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plot3DCurveColormap(curve)

    hold on;
    
    npoints = size(curve,1);
    
    map = colormap(jet(npoints));
    
    for i=1:npoints-1
        plot3(curve(i:i+1,1),... 
              curve(i:i+1,2),...
              curve(i:i+1,3),...
              '-','Color',map(i,:),...
              'LineWidth',2);
    end
    
return


