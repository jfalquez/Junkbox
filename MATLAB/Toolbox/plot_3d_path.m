% draw a robot trajectory
% rpath is a 6xN vector of 3D poses
function h = plot_3d_path(rpath, scale)

    if(nargin == 1)
        scale = 1;
    end

    hold on; 
    axis equal;
    grid on;

    xlabel('x');
    ylabel('y');
    zlabel('z');

    h = [];
    for ii = 1:size(rpath, 2)
        h = [h, plot_3d_cf(rpath(:,ii), scale)];
    end


