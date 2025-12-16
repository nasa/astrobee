function fStar = dualFunction( H, A, zL, zU, lambdagrid  )

    nElem = size(lambdagrid,2);
    
    zL_ = repmat( zL,1,nElem );
    zU_ = repmat( zU,1,nElem );

    % dual function
    z_uc = - H^-1 * A'*lambdagrid;
    zStar = max(zL_,min(z_uc,zU_));

    fStar = zeros(1,nElem);
    for i = 1:nElem
        z = zStar(:,i);
        lambda = lambdagrid(:,i);
        fStar(i) = .5*z'*H*z + lambda' * A * z;
    end