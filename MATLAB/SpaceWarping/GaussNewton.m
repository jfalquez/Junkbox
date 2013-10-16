%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x = GaussNewton( varargin )
    
    f = varargin{1};
    x = varargin{2};
    
    MaxIter = 10;
    Eps = 1e-6;
    
    for ii = 1:MaxIter
        
        [r,J] = feval(f,x,varargin{3:end});
        
        A = J'*J;
        b = J'*r;
        
%        if( rank(full(A)) < size(A,1))
            Lambda = 1e-6*eye(size(A));
            xnew = x - (A+Lambda)\b;            
%        else
%            xnew = x - A\b;
%        end

%        fprintf('Iteration %d: norm %f\n', ii, norm(x-xnew) );
        
        if( norm(x-xnew) < Eps )
            return;
        end
        x = xnew;
    end
end