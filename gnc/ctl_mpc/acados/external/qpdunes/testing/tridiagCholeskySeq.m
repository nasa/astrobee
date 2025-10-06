function [cholM] = tridiagCholeskySeq( M, sizeMatrix, blckSize )

dim0 = sizeMatrix;

cholM = zeros(sizeMatrix,sizeMatrix);

for ii=1:dim0
		%get block start and end ids
		rowStart = max( (floor(ii / blckSize)-1) * blckSize +1, 1 );
		columnEnd = min( (floor(ii / blckSize)+2) * blckSize, dim0 );
		
		%write diagonal element: jj == ii
		sum = M(ii,ii);
		
        for kk=rowStart:ii
            %subtract squared forepart of this row
            sum = sum-cholM(ii,kk) * cholM(ii,kk);
        end
        
        cholM(ii,ii) = sqrt( sum );
		
		% write remainder of ii-th column
        for jj=(ii+1):columnEnd
			sum =  M(jj,ii);
			
            for kk=rowStart:ii  % subtract forepart of this row times forepart of ii-th row */
				sum = sum - cholM(ii,kk) * cholM(jj,kk);
            end
			
			cholM(jj,ii) = sum / cholM(ii,ii);
        end
end


