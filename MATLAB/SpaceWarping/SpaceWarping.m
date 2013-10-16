function SpaceWarping( z )

    nsteps = numel(unique(z(3,:)));

    % Init
    ctx.landmarks = {};
    ctx.poses{1}.x =  [0;0;0];
    ctx.zi{1} = z( :, find( z(3,:) == 1 ) ); % current measurements
    ctx = StartNewLandmarks( ctx.poses{1}.x, ctx.zi{1}, ctx );
    
    for ii = 2:nsteps
        ctx.zi{ii} = z( :, find( z(3,:) == ii ) ); % current measurements
        ctx.poses{ii}.x = ctx.poses{ii-1}.x; % zero motion model
        fprintf('Front End: ');
        tic;
        ctx = ComputeMotion( ctx.poses{ii}.x, ctx.zi{ii}, ctx );
        ctx = StartNewLandmarks( ctx.poses{ii}.x, ctx.zi{ii}, ctx );
        fprintf('%f\n', toc);
        fprintf('Back End: ');
        tic;
        ctx = SolveFullSlam( ctx );
        fprintf('%f\n', toc);
        
        ctx = PlotResults( ctx );
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ctx = PlotResults( ctx )
    if isfield(ctx,'h')
        delete(ctx.h)
    end
    [p,m] = PathAndMap( ctx );
    ctx.h = plot_2d_path( p, 0.1 );
    ctx.h = [ctx.h plot( m(1,:), m(2,:), 'x' )];
    drawnow();
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [p,map] = PathAndMap( ctx )
    lmidxs = find( cellfun(@isempty,ctx.landmarks) == 0 );
    n = numel(lmidxs);
    m = size(ctx.poses,2);
    p = zeros(3,m);
    for jj = 1:m
        p(:,jj) = ctx.poses{jj}.x;
    end
    map = zeros(2,n);
    for ii = 1:n
       map(:,ii) = ctx.landmarks{lmidxs(ii)}.x;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ctx = SolveFullSlam( ctx )

    ctx.sw = 10; % sliding window size
    ctx.active_idx_start = max( size(ctx.poses,2)-ctx.sw, 1 );
    ctx.active_idx_end   = min( ctx.active_idx_start+ctx.sw, size(ctx.poses,2) );
%     ctx.active_idx_start = 1;
%     ctx.active_idx_end   = size(ctx.poses,2);
    ctx.static_idx_start = max( ctx.active_idx_start-5, 1 );
    ctx.static_idx_end   = ctx.active_idx_start-1;
%     ctx.static_idx_start = 2;
%     ctx.static_idx_end   = 1;


    [ctx,x] = GetState(ctx);
    x = GaussNewton( @(x)SlamResidualWithJacobian(x,ctx), x );
    ctx = SetState(x,ctx);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ctx,x] = GetState(ctx)
    x = [];
    ctx.num_measurements = 0;
    ctx.lmk_idxs = [];
    for ii = ctx.static_idx_start:ctx.static_idx_end
        ctx.poses{ii}.state_idx = numel(x)+1;
        x = [x; ctx.poses{ii}.x];
        ctx.num_measurements = ctx.num_measurements + size(ctx.zi{ii},2);
        ctx.lmk_idxs = [ctx.lmk_idxs ctx.zi{ii}(4,:)];
    end
    for ii = ctx.active_idx_start:ctx.active_idx_end
        ctx.poses{ii}.state_idx = numel(x)+1;
        x = [x; ctx.poses{ii}.x ];
        ctx.num_measurements = ctx.num_measurements + size(ctx.zi{ii},2);
        ctx.lmk_idxs = [ctx.lmk_idxs ctx.zi{ii}(4,:)];
    end
    ctx.lmk_idxs = unique(ctx.lmk_idxs);
%     ctx.lmk_idxs = find( cellfun(@isempty,ctx.landmarks) == 0 );
    for jj = 1:numel( ctx.lmk_idxs );
        ctx.landmarks{ctx.lmk_idxs(jj)}.state_idx = numel(x)+1;
        x = [x; ctx.landmarks{ctx.lmk_idxs(jj)}.x];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ctx = SetState(x,ctx)
    for ii = ctx.active_idx_start:ctx.active_idx_end
        idx = ctx.poses{ii}.state_idx;
        ctx.poses{ii}.x = x(idx:idx+2);
    end
%     ctx.lmidxs = find( cellfun(@isempty,ctx.landmarks) == 0 );
    for jj = 1:numel( ctx.lmk_idxs );
        idx = ctx.landmarks{ctx.lmk_idxs(jj)}.state_idx;
        ctx.landmarks{ctx.lmk_idxs(jj)}.x = x(idx:idx+1);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [r,J] = SlamResidualWithJacobian( x, ctx )
%    r = SlamResidual( x, ctx );
%    J = FiniteDiff( @(x)SlamResidual(x,ctx), x );
    [r,J] = SlamResidualJacobian(x,ctx);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [r,J] = SlamResidualJacobian( x, ctx )
    J = sparse(2*ctx.num_measurements,numel(x));
    r = zeros(2*ctx.num_measurements,1);
    ri = 1;

    for ii = ctx.static_idx_start:ctx.static_idx_end
        zi = ctx.zi{ii};
        pose_stateidx = ctx.poses{ii}.state_idx;
        xwpi = x(pose_stateidx:pose_stateidx+2);
        for cc = 1:size(zi,2)
            lmidx = zi(4,cc);
            lmk_stateidx = ctx.landmarks{lmidx}.state_idx;
            xwmj = x(lmk_stateidx:lmk_stateidx+1);
            zij = zi(1:2,cc);
            
            [hij,Jwpi,Jwmj] = SensorModel( xwpi, xwmj );
            r(ri:ri+1) = zij-hij;
            J(ri:ri+1,lmk_stateidx:lmk_stateidx+1) = -Jwmj;
            ri = ri+2;
        end
    end
    for ii = ctx.active_idx_start:ctx.active_idx_end
        zi = ctx.zi{ii};
        pose_stateidx = ctx.poses{ii}.state_idx;
        xwpi = x(pose_stateidx:pose_stateidx+2);
        for cc = 1:size(zi,2)
            lmidx = zi(4,cc);
            lmk_stateidx = ctx.landmarks{lmidx}.state_idx;
            xwmj = x(lmk_stateidx:lmk_stateidx+1);
            zij = zi(1:2,cc);
            
            [hij,Jwpi,Jwmj] = SensorModel( xwpi, xwmj );
            r(ri:ri+1) = zij-hij;
            J(ri:ri+1,pose_stateidx:pose_stateidx+2) = -Jwpi;
            J(ri:ri+1,lmk_stateidx:lmk_stateidx+1) = -Jwmj;
            ri = ri+2;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function r = SlamResidual( x, ctx )
    r = [];
    for ii = 1:size(ctx.poses,2)
        zi = ctx.zi{ii};
        xwpi = x(ii*3-2:ii*3);
        for cc = 1:size(zi,2)
            lmidx = zi(4,cc);
            stateidx = ctx.landmarks{lmidx}.state_idx;
            xwmj = x(stateidx:stateidx+1);
            zij = zi(1:2,cc);
            hij = SensorModel( xwpi, xwmj );
            r = [ r; zij-hij];
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ctx = ComputeMotion( xwpi, zi, ctx )
    ctx.z = [];
    for jj = 1:size(zi,2)
        lmidx = zi(4,jj);
        zij = zi(:,jj);
        if lmidx <= size(ctx.landmarks,2) && ~isempty( ctx.landmarks{lmidx} )
            ctx.z = [ ctx.z zij ];
        end
    end

    if( isempty(ctx.z) )
        ctx.poses{end}.x = [0;0;0];
    else
        x = GaussNewton( @(x)MotionResidualWithJacobian(x,ctx), xwpi );
        ctx.poses{end}.x = x;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [r,J] = MotionResidualWithJacobian( x, ctx )
    r = MotionResidual( x, ctx );
    J = FiniteDiff( @(x)MotionResidual(x,ctx), x );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function r = MotionResidual( x, ctx )
    r = [];
    for jj = 1:size(ctx.z,2)
        zij = ctx.z(1:2,jj);
        lmidx = ctx.z(4,jj);
        hij = SensorModel( x, ctx.landmarks{lmidx}.x );
        r = [ r; zij-hij ]; 
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [hij,Jxwr,Jxwj] = SensorModel( xwr, xwj )

    xrj = tcomp( tinv(xwr), [xwj;1] );
    range = norm(xrj(1:2));
    bearing = atan2( xrj(2), xrj(1) );
    hij = [range;bearing];

    if nargout == 1
        return
    end
    
%    dnorm = Jnorm( xrj );
%    datan = Jatan( xrj );
%    [dtcomp_ab,dtcomp_bc] = Jtcomp(tinv(xwr),xwj);
%    dtinv = Jtinv(xwr);

%    Jxwr = [dnorm*dtcomp_ab*dtinv;...
%        datan*dtcomp_ab*dtinv];

% 	if nargout == 2
%         return
%     end

%    Jxwj = [dnorm*dtcomp_bc;...
%        datan*dtcomp_bc];
    
    
    % holy balls batman, matlab says this is the same. wtf goes the
    % trig???!
    xab = xwr(1);
    yab = xwr(2);
    xbc = xwj(1);
    ybc = xwj(2);
    t = (xab^2 - 2*xab*xbc + xbc^2 + yab^2 - 2*yab*ybc + ybc^2);
    J = [ (xab - xbc)/t^(1/2), (yab - ybc)/t^(1/2),  0, -(xab - xbc)/t^(1/2), -(yab - ybc)/t^(1/2);...
        -(yab - ybc)/t,       (xab - xbc)/t, -1,        (yab - ybc)/t,       -(xab - xbc)/t];

    Jxwr = J(:,1:3);
    Jxwj = J(:,4:5);
end

function ctx = StartNewLandmarks( xwpi, zi, ctx )
    for jj = 1:size(zi,2)
        lmidx = zi(4,jj);
        if( lmidx > size(ctx.landmarks,2) || isempty(ctx.landmarks{lmidx}) )
            ctx.landmarks{lmidx}.x = GetWorldPos( xwpi, zi(:,jj) );
        end
    end
end

function xwmj = GetWorldPos( xwpi, zij )
    theta = xwpi(3)+zij(2);
    range = zij(1);
    x = range*cos(theta) + xwpi(1);
    y = range*sin(theta) + xwpi(2);
    xwmj = [x;y];
end

function [Jab,Jbc] = Jtcomp( tab, tbc )
    cth = cos(tab(3));
    sth = sin(tab(3));
    Jab = [ 1, 0, - cth*tbc(2) - sth*tbc(1);...
            0, 1,   cth*tbc(1) - sth*tbc(2);...
            0, 0,                         1];
    if nargout == 2
        Jbc = [cth, -sth;...
               sth,  cth;...
               0,    0];
    end
end

function J = Jnorm(tab)
    xab = tab(1);
    yab = tab(2);
     J = [ xab/(xab^2 + yab^2)^(1/2), yab/(xab^2 + yab^2)^(1/2), 0 ];
end

function J = Jatan(tab)
    xab = tab(1);
    yab = tab(2);
    J = [ -yab/(xab^2 + yab^2), xab/(xab^2 + yab^2), 0];
end

function J = Jtinv(tab)
    cth = cos(tab(3));
    sth = sin(tab(3));
    J = [ -cth, -sth, sth*tab(1) - cth*tab(2);...
	       sth, -cth, cth*tab(1) + sth*tab(2);...
             0,    0,                -1];
end