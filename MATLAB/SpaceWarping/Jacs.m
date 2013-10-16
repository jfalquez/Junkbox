function Jacs
%    syms x y th lmx lmy;
%    res = SensorModel( [x;y;th], [lmx;lmy] );
%    simplify( jacobian( res, [x,y,th,lmx,lmy] ) )

    syms xab yab thab xbc ybc;
    xwr = [xab;yab;thab];
    xwj = [xbc;ybc];

    xrj = mytcomp( tinv(xwr), xwj );

    dnorm = Jnorm( xrj );
    datan = Jatan( xrj );
    [dtcomp_ab,dtcomp_bc] = Jtcomp(tinv(xwr),xwj);
    dtinv = Jtinv(xwr);
    Jxwr = [dnorm*dtcomp_ab*dtinv;...
            datan*dtcomp_ab*dtinv];
    Jxwj = [dnorm*dtcomp_bc;...
            datan*dtcomp_bc];

    res = SensorModel( xwr, xwj );
    a = simplify( jacobian( res, [xwr;xwj] ) )
    b = simplify( [Jxwr,Jxwj] )
    ccode(b)
end

function hij = SensorModel( xwr, xwj )
    xrj = mytcomp( tinv(xwr), [xwj;1] );
    range = sqrt( xrj(1:2).'*xrj(1:2) );
    bearing = atan( xrj(2)/xrj(1) );
    hij = [range;bearing];
end


function tac = mytcomp( tab, tbc )
%    result = tab(3)+tbc(3);
    s = sin(tab(3));
    c = cos(tab(3));
    tac(1,1) = tab(1) + c*tbc(1) - s*tbc(2);
    tac(2,1) = tab(2) + s*tbc(1) + c*tbc(2);
    %tac(3,1) = result;
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
