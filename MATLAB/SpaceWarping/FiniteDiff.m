function J = FiniteDiff( varargin )

    f = varargin{1};
    x = varargin{2};
    
    dEps = 1e-4;
    
    y = feval( f, x, varargin{3:end} );
    
    J = zeros( numel(y), numel(x) );
    
    for( ii = 1:numel(x) )
    
        xPlus = x;
        xPlus(ii) = xPlus(ii) + dEps;
        yPlus = feval( f, xPlus, varargin{3:end} );
        
        xMinus = x;
        xMinus(ii) = xMinus(ii) - dEps;
        yMinus = feval( f, xMinus, varargin{3:end} );

        J(:,ii) = (yPlus-yMinus)./(2*dEps);
    end
