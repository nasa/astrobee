function x = parallelFactorization( H, b, N, nX )
    
    % initialization
    D = cell( ceil(log2(N-1))+1 +1 , N ); 
    U = cell( ceil(log2(N-1))+1 +1 , N-1 ); 
    rhs = cell( ceil(log2(N-1))+1 +1 , N );
%     DD = cell( N ); 
%     UU = cell( N-1 ); 
%     rhsNew = cell( N );
    res = cell( ceil(log2(N-1))+1 +1 );
    idxList = cell( ceil(log2(N-1))+1 +1 , N );
    
    
    for kk = 1:N-1
        D{1, kk} = H( 1+(kk-1)*nX:kk*nX , 1+(kk-1)*nX:kk*nX );
        U{1, kk} = H( 1+(kk-1)*nX:kk*nX , 1+kk*nX:(kk+1)*nX );
        rhs{1, kk} = b( 1+(kk-1)*nX:kk*nX );
%         DD{kk} = H( 1+(kk-1)*nX:kk*nX , 1+(kk-1)*nX:kk*nX );
%         UU{kk} = H( 1+(kk-1)*nX:kk*nX , 1+kk*nX:(kk+1)*nX );
%         rhsNew{kk} = b( 1+(kk-1)*nX:kk*nX );
    end
    D{1, N} = H( 1+(N-1)*nX:N*nX , 1+(N-1)*nX:N*nX );
    rhs{1, N} = b( 1+(N-1)*nX:N*nX );
%     DD{N} = H( 1+(N-1)*nX:N*nX , 1+(N-1)*nX:N*nX );
%     rhsNew{N} = b( 1+(N-1)*nX:N*nX );
    
    Njj_list = zeros( ceil(log2(N-1))+1 +1 , 1);
    Njj_list(1) = N;
    
    for kk = 1:N
        idxList{1, kk} = kk;
    end
    
    
    % elimination order
    eliminationOrder = zeros(N,4);
    elimIdx = 0;
    elimLevel = 0;
    kkStep = 1;
    while elimIdx < N-1
        elimLevel = elimLevel + 1;
        kkStart = 1+kkStep;
        kkStep = kkStep * 2;
        for kk = kkStart:kkStep:N
            elimIdx = elimIdx + 1;
            eliminationOrder(elimIdx,1) = kk;
            eliminationOrder(elimIdx,2) = kk-kkStep/2;   % left parent  % > 0 by definition
            if (kk+kkStep/2 <= N)
                eliminationOrder(elimIdx,3) = kk+kkStep/2;   % right parent
            else
                eliminationOrder(elimIdx,3) = 0;
            end
            eliminationOrder(elimIdx,4) = elimLevel;    % elimination level
        end        
    end
    eliminationOrder(N,1) = 1;
    eliminationOrder(N,2) = 0;              % left parent
    eliminationOrder(N,3) = 0;              % right parent
    eliminationOrder(N,4) = elimLevel+1;    % elimination level
    
    
    %% factor step
%     for jj = 1 : ceil(log2(N-1))+1
%         
%         newIdx = 0;
%        
%         for kk = 1:2:Njj_list(jj)
%             newIdx = newIdx + 1;
%             
%             idxList{jj+1, newIdx} = idxList{jj, kk};            
%             
%             % build new diagonal block from generalized Schur complement
%             if kk == 1
%                 % build diagonal block
%                 D{jj+1,newIdx} = D{jj,kk} - U{jj,kk} * ( D{jj,kk+1} \ U{jj,kk}' );
%                 % correct rhs
%                 rhs{jj+1,newIdx} = rhs{jj,kk} - U{jj,kk} * ( D{jj,kk+1} \ rhs{jj,kk+1} );
%             elseif kk+1 <= Njj_list(jj)
%                 % build diagonal block
%                 D{jj+1,newIdx} = D{jj,kk} - U{jj,kk-1}' * ( D{jj,kk-1} \ U{jj,kk-1} ) ...
%                                           - U{jj,kk} * ( D{jj,kk+1} \ U{jj,kk}' );
%                 % rhs
%                 rhs{jj+1,newIdx} = rhs{jj,kk} - U{jj,kk-1}' * ( D{jj,kk-1} \ rhs{jj,kk-1} ) ...
%                                           - U{jj,kk} * ( D{jj,kk+1} \ rhs{jj,kk+1} );
%             else
%                 % build diagonal block
%                 D{jj+1,newIdx} = D{jj,kk} - U{jj,kk-1}' * ( D{jj,kk-1} \ U{jj,kk-1} );
%                 % rhs
%                 rhs{jj+1,newIdx} = rhs{jj,kk} - U{jj,kk-1}' * ( D{jj,kk-1} \ rhs{jj,kk-1} );
%             end
%             
%             
%             % superdiagonal blocks
%             if kk+1 <= Njj_list(jj)-1    
%                 % build new superdiagonal block
%                 U{jj+1,newIdx} = - U{jj,kk} * ( D{jj,kk+1} \ U{jj,kk+1} );
%             end
% 
%         end
%         
%         Njj_list(jj+1) = newIdx;
%         
%         if Njj_list(jj+1) == 1
%             break;
%         end
% 
%     end %/* next elimination level */


    for ii = 1:N
        elimIdx = eliminationOrder(ii,1);
        leftParent = eliminationOrder(ii,2);
        rightParent = eliminationOrder(ii,3);
        jj = eliminationOrder(ii,4);
        
            % build new diagonal block from generalized Schur complement
            if (leftParent > 0)     % left parent exists
                % build diagonal block
                D{jj+1,leftParent} = D{jj,leftParent} - U{jj,leftParent} * ( D{jj,elimIdx} \ U{jj,leftParent}' );
                % correct rhs
                rhsNew{leftParent} = rhsNew{leftParent} - U{jj,leftParent} * ( D{jj,elimIdx} \ rhsNew{elimIdx} );
                % build superdiagonal block
                elimIdx
%                 DD{elimIdx}
%                 UU{elimIdx}
                if (rightParent > 0)    % check whether elimIdx is last remaining index; then UU doesn't exist
                    UU{leftParent} = - U{jj,leftParent} * ( D{jj,elimIdx} \ U{jj,elimIdx} );
                end
            end
            if (rightParent > 0)     % left parent exists
                % build diagonal block
                D{jj+1,rightParent} = D{jj,rightParent} - U{jj,elimIdx}' * ( D{jj,elimIdx} \ U{jj,elimIdx} );
                % rhs
                rhsNew{rightParent} = rhsNew{rightParent} - U{jj,elimIdx}' * ( D{jj,elimIdx} \ rhsNew{elimIdx} );
%                 % build superdiagonal block
%                 if (rightParent <= N-1)
%                     UU{elimIdx} = - UU{leftParent} * ( DD{elimIdx} \ UU{elimIdx} );
%                 end
            end
                
%             elseif kk+1 <= Njj_list(jj)
%                 % build diagonal block
%                 D{jj+1,newIdx} = D{jj,kk} - U{jj,kk-1}' * ( D{jj,kk-1} \ U{jj,kk-1} ) ...
%                                           - U{jj,kk} * ( D{jj,kk+1} \ U{jj,kk}' );
%                 % rhs
%                 rhs{jj+1,newIdx} = rhs{jj,kk} - U{jj,kk-1}' * ( D{jj,kk-1} \ rhs{jj,kk-1} ) ...
%                                           - U{jj,kk} * ( D{jj,kk+1} \ rhs{jj,kk+1} );
%             else
%                 % build diagonal block
%                 D{jj+1,newIdx} = D{jj,kk} - U{jj,kk-1}' * ( D{jj,kk-1} \ U{jj,kk-1} );
%                 % rhs
%                 rhs{jj+1,newIdx} = rhs{jj,kk} - U{jj,kk-1}' * ( D{jj,kk-1} \ rhs{jj,kk-1} );
%             end
            
            
%             % superdiagonal blocks
%             if kk+1 <= Njj_list(jj)-1    
%                 % build new superdiagonal block
%                 U{jj+1,newIdx} = - U{jj,kk} * ( D{jj,kk+1} \ U{jj,kk+1} );
%             end
    end
    
    
    %% backsolve step
    
    disp('done with factorization, starting backsolve now');

%     for jj = ceil(log2(N-1))+1 : -1 : 1
%         
%         if Njj_list(jj) == 0
%             continue;
%         end
%         
%         
%         for kk = 1:Njj_list(jj)
% 
%             newIdx = idxList{jj,kk};
%             
%             % skip already resolved indices
%             if sum(ismember([idxList{jj+1,:}],newIdx)) > 0
%                 continue;
%             end
            

    for kk = N:-1:1
            newIdx = eliminationOrder(kk,1);
            lowerIdx = eliminationOrder(kk,2);
            upperIdx = eliminationOrder(kk,3);
            elimLevel = eliminationOrder(ii,4);
            
            % correct for substituted variables
%             if kk > 1
%             
%                 lowerIdx = idxList{jj,kk-1};
            if lowerIdx ~= 0
%                 rhs{jj,kk} = rhs{jj,kk} - U{jj,kk-1}' * res{lowerIdx};
                rhsNew{newIdx} = rhsNew{newIdx} - U{elimLevel,lowerIdx}' * res{lowerIdx};
                
            end
                
%             if kk+1 <= Njj_list(jj)
%             
%                 upperIdx = idxList{jj,kk+1};
            if upperIdx ~= 0
%                 rhs{jj,kk} = rhs{jj,kk} - U{jj,kk} * res{upperIdx};
                rhsNew{newIdx} = rhsNew{newIdx} - U{elimLevel,newIdx} * res{upperIdx};
                
            end

%             res{newIdx} = D{jj,kk} \ rhs{jj,kk};
            res{newIdx} = D{elimLevel,newIdx} \ rhsNew{newIdx};

%         end

    end %/* next elimination level */
    

    
    % reshape result
    x = zeros(N * nX, 1);
   
    for kk = 1:N
        x( 1+(kk-1)*nX:kk*nX ) = res{kk};
    end
    
    
    
    
    
    