function [z,p,m] = GenSim()

    clear all;
    close all;
    
    randn('seed',1);
    rand('seed',1);

    nlandmarks = 1000;
    startpose = [ 0; 0; 0];
    max_range = 5;
    fov = pi/2;
    percent_to_keep = 0.9;

    p = GenPath( startpose );

    m = GenMap( p, nlandmarks, max_range );

    hold on;
    plot_2d_path( p, 0.1 );
    plot( m(1,:), m(2,:), 'x' );

    z = GenMeasurements( p, m, max_range, fov, percent_to_keep );
end    

   
function m = GenMap( path, nlandmarks, max_range )

    minx = min(path(1,:));
    maxx = max(path(1,:));
    rangex = (maxx-minx)+2*max_range;
    meanx = mean( path(1,:) );
    miny = min(path(2,:));
    maxy = max(path(2,:));
    rangey = (maxy-miny)+2*max_range;
    meany = mean( path(2,:) );

    x = (rangex*rand(1,nlandmarks) - rangex/2) + meanx;
    y = (rangey*rand(1,nlandmarks) - rangey/2) + meany;
    
    m = [x;y];
end


function p = GenPath( startpose );
    p = MakeMotion( startpose, [0.1;0;0], 100 );% forward motion
    p = [p MakeMotion( p(:,end), [0.1;0;(pi/2)/20], 20 )];% right turn
    p = [p MakeMotion( p(:,end), [0.1;0;0], 100 )];% forward motion
    p = [p MakeMotion( p(:,end), [0.1;0;(pi/2)/20], 20 )];% right turn
	p = [p MakeMotion( p(:,end), [0.1;0;0], 100 )];% forward motion
    p = [p MakeMotion( p(:,end), [0.1;0;(pi/2)/20], 20 )];% right turn
    p = [p MakeMotion( p(:,end), [0.1;0;0], 100 )];% forward motion
    p = [p MakeMotion( p(:,end), [0.1;0;(pi/2)/20], 20 )];% right turn
end

function p = MakeMotion( basepose, dpose, n )
    p = basepose;
    for ii = 1:n
        p = [ p tcomp(p(:,end),dpose)];
    end
end


function z = GenMeasurements( p, m, max_range, fov, percent_to_keep )
    z = [];
    for ii = 1:size(p,2)
       zi = GenMeasurement( p(:,ii), ii, m, max_range, fov, percent_to_keep );
       z = [z zi];
    end
end

function zi = GenMeasurement( x_wr, timestep, m, max_range, fov, percent_to_keep )

    x_rpl = max_range*[ cos(fov/2);  sin(fov/2) ];
    x_rpr = max_range*[ cos(fov/2); -sin(fov/2) ];
    
    x_wpl = tcomp( x_wr, [x_rpl;0] );
    x_wpr = tcomp( x_wr, [x_rpr;0] );

    h = [];
    h = [ h plot( [x_wr(1), x_wpl(1)],  [x_wr(2), x_wpl(2)], 'r-' )];
    h = [ h plot( [x_wr(1), x_wpr(1)],  [x_wr(2), x_wpr(2)], 'r-' )];
    
    cx = m(1,:) - x_wr(1);
    cy = m(2,:) - x_wr(2); 
    dx = cx.^2;
    dy = cy.^2;
    
    d = [dx+dy];

    idx = find( d < max_range^2 );

    zi = [];
    for ii = 1:numel(idx)
        xwpi = m(:,idx(ii));

        xrp = tcomp( tinv(x_wr), [xwpi;0] );
        range   = norm(xrp(1:2));
        bearing = atan2( xrp(2), xrp(1) );

        keep = rand;
        if( abs(bearing) < fov/2 && keep < percent_to_keep )
            h = [ h plot( [x_wr(1),xwpi(1)],  [x_wr(2),xwpi(2)], 'g-' )];    
            zi = [zi [range; bearing; timestep; idx(ii)] ];
        end
    end
    drawnow();
    delete(h);
end
