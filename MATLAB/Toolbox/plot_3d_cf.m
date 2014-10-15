%
% h = plot_3d_cf(pose, scale) 
%
function h = plot_3d_cf(pose, scale)
    if(nargin == 1)
        scale = 1;
    end
    
    T = Cart2T(pose);
    
    R = T(1:3, 1:3);
    
    center = [T(1, 4), T(2, 4), T(3, 4)];
   
    X =  center +  (R(:,1)' .* scale);
    Y =  center +  (R(:,2)' .* scale);
    Z =  center +  (R(:,3)' .* scale);
    
    h(1) = line( [center(1), X(1)], [center(2), X(2)], [center(3), X(3)], 'Color', 'r', 'LineWidth', 1);
    h(2) = line( [center(1), Y(1)], [center(2), Y(2)], [center(3), Y(3)], 'Color', 'g', 'LineWidth', 1);
    h(3) = line( [center(1), Z(1)], [center(2), Z(2)], [center(3), Z(3)], 'Color', 'b', 'LineWidth', 1);