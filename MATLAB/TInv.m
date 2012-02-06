function invT = TInv( T )
        invT(1:3,1:3) =  transpose(T(1:3,1:3));
        invT(1:3,4) = -transpose(T(1:3,1:3)) * T(1:3,4);
        invT(4,1:4) = [0 0 0 1]; 
