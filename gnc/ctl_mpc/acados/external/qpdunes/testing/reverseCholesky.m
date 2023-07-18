function cholH = reverseCholesky( H )

    cholH = zeros(size(H));
    nX = length(H);

    for jj= nX:-1:1
        %/* 1) compute diagonal element: ii == jj */
        %/* take diagonal element of original */
        sum = H(jj,jj);

        %/* subtract squared rearpart of corresponding row (transposed access, therefore rest of column): */
        for  ll=jj+1:nX
            sum = sum - cholH(ll,jj) * cholH(ll,jj);	%/* transposed access */
        end


        %/* 2) check for too small diagonal elements */
        if (sum < 1e-10)
            sum = sum + 1e-6;
            display 'cholesky warning: needed to regularize!'
        end
        cholH(jj,jj) = sqrt( sum );


        %/* 3) write remainder of jj-th column (upwards! via transposed access: jj-th row, leftwards): */
        for ii = jj-1:-1:1
            sum = H(jj,ii);	%/* transposed access */

            %/* subtract rearpart of this row times rearpart of jj-th row */
            for ll = jj+1:nX
                sum = sum - cholH(ll,ii) * cholH(ll,jj);
            end

            %/* write transposed! (otherwise it's upper triangular matrix) */
            %cholH(ii,jj) = sum / cholH(jj,jj);
            cholH(jj,ii) = sum / cholH(jj,jj);
        end
    end %/* next column */