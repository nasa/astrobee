/*
 *	This file is part of qpDUNES.
 *
 *	qpDUNES -- A DUal NEwton Strategy for convex quadratic programming.
 *	Copyright (C) 2012 by Janick Frasch, Hans Joachim Ferreau et al. 
 *	All rights reserved.
 *
 *	qpDUNES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpDUNES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpDUNES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file src/matrix_vector.c
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */


#include "qp/matrix_vector.h"

/* ---------------------------------------------- 
 * Matrix-vector product y = Q*x
 * 
 >>>>>                                            */
return_t multiplyQx( qpData_t* const qpData,
					 x_vector_t* const res,
					 const xx_matrix_t* const Q,
					 const x_vector_t* const x 	)
{
	return multiplyMatrixVector( (vector_t*)res, (matrix_t*)Q, (vector_t*)x, _NX_, _NX_ );
}
/*<<< END OF multiplyQx */


/* ---------------------------------------------- 
 * Matrix-vector product x = Qinv*y
 * 
 > >>>>                    *                        */
return_t multiplyInvQx(	qpData_t* const qpData,
						x_vector_t* const res,
						const xx_matrix_t* const cholQ,
						const x_vector_t* const y 	)
{
	return multiplyMatrixVector( (vector_t*)res, (matrix_t*)cholQ, (vector_t*)y, _NX_, _NX_ );
}
/*<<< END OF multiplyQinvy */


/* ----------------------------------------------
 * Matrix-vector product y = R*u
 * 
 >>>>>                                            */
return_t multiplyRu( qpData_t*const  qpData,
					 u_vector_t* const res,
					 const uu_matrix_t* const R,
					 const u_vector_t* const u 	)
{
	return multiplyMatrixVector( (vector_t*)res, (matrix_t*)R, (vector_t*)u, _NU_, _NU_ );
}
/*<<< END OF multiplyRu */


/* ----------------------------------------------
 * Matrix-vector product u = Rinv*y
 * 
 > >>>>                    *                        */
return_t multiplyInvRu(	qpData_t* const qpData,
						u_vector_t* const res,
						const uu_matrix_t* const cholR,
						const u_vector_t* const y 	)
{
	return multiplyMatrixVector( (vector_t*)res, (matrix_t*)cholR, (vector_t*)y, _NU_, _NU_ );
}
/*<<< END OF multiplyRinvu */


/* ----------------------------------------------
 * Matrix-vector product y = z'*H*z
 * 
 >>>>>                                            */
real_t multiplyzHz( qpData_t* const qpData,
					 const vv_matrix_t* const H,
					 const z_vector_t* const z,
					 const int_t nV	)
{
	return multiplyVectorMatrixVector( (matrix_t*)H, (vector_t*)z, nV );
}
/*<<< END OF multiplyzHz */


/* ----------------------------------------------
 * Matrix-vector product y = invH*z
 * 
 >>>>>                                            */
return_t multiplyInvHz(	qpData_t* const qpData,
						z_vector_t* const res,
						const vv_matrix_t* const cholH,
						const z_vector_t* const z,
						const int_t nV )
{
	return multiplyInvMatrixVector( qpData, (vector_t*)res, (matrix_t*)cholH, (vector_t*)z, nV );
}
/*<<< END OF multiplyInvHz */


/* ----------------------------------------------
 * Matrix-vector product res = A*x
 * 
 >>>>>                                            */
return_t multiplyAx(	qpData_t* const qpData,
						x_vector_t* const res,
						const xx_matrix_t* const A,
						const x_vector_t* const x 	)
{
	return multiplyMatrixVector( (vector_t*)res, (matrix_t*)A, (vector_t*)x, _NX_, _NX_ );
}
/*<<< END OF multiplyAx */


/* ----------------------------------------------
 * Matrix-vector product res = B*u
 * 
 >>>>>                                            */
return_t multiplyBu(	qpData_t* const qpData,
						x_vector_t* const res,
						const xu_matrix_t* const B,
						const u_vector_t* const u 	)
{
	return multiplyMatrixVector( (vector_t*)res, (matrix_t*)B, (vector_t*)u, _NX_, _NU_ );
}
/*<<< END OF multiplyBu */


/* ----------------------------------------------
 * Matrix-vector product res = C*z
 * 
 >>>>>                                            */
return_t multiplyCz(	qpData_t* const qpData,
						x_vector_t* const res,
						const xz_matrix_t* const C,
//						const xx_matrix_t* const A,
//						const xu_matrix_t* const B,
						const z_vector_t* const z 	)
{
	/** only dense multiplication */
	int_t ii, jj;
	
	for( ii = 0; ii < _NX_; ++ii ) {
		res->data[ii] = 0.;
		for( jj = 0; jj < _NZ_; ++jj ) {
			res->data[ii] += accC(ii,jj) * z->data[jj];
		}
	}
	
	return QPDUNES_OK;
}
/*<<< END OF multiplyCz */


///* ----------------------------------------------
// * Matrix-vector product x = A.T*y
// *
// >>>>>                                            */
//return_t multiplyATy( qpData_t* const qpData,
//					  x_vector_t* const res,
//					  const xx_matrix_t* const A,
//					  const x_vector_t* const y 	)
//{
//	return multiplyMatrixTVector( (vector_t*)res, (matrix_t*)A, (vector_t*)y, _NX_, _NX_ );
//}
///*<<< END OF multiplyATy */
//
//
///* ----------------------------------------------
// * Matrix-vector product x = B.T*y
// *
// >>>>>                                            */
//return_t multiplyBTy(	qpData_t* const qpData,
//						u_vector_t* const res,
//						const xu_matrix_t* const B,
//						const x_vector_t* const y 	)
//{
//	return multiplyMatrixTVector( (vector_t*)res, (matrix_t*)B, (vector_t*)y, _NX_, _NU_ );
//}
///*<<< END OF multiplyBTy */


/* ----------------------------------------------
 * Matrix-vector product z = C.T*y
 * 
 >>>>>                                            */
return_t multiplyCTy(	qpData_t* const qpData,
						z_vector_t* const res,
						const xz_matrix_t* const C,
//						const xx_matrix_t* const A,
//						const xu_matrix_t* const B,
						const x_vector_t* const y 	)
{
	/** only dense multiplication */
	int_t ii, jj;
	
	/* change multiplication order for more efficient memory access */
	for( jj = 0; jj < _NZ_; ++jj ) {
		res->data[jj] = 0.;
	}
	for( ii = 0; ii < _NX_; ++ii ) {
		for( jj = 0; jj < _NZ_; ++jj ) {
			res->data[jj] += accC(ii,jj) * y->data[ii];
		}
	}
	
	return QPDUNES_OK;
}
/*<<< END OF multiplyCTy */


///* ----------------------------------------------
// * Inverse matrix times matrix product res = Q^-1 * A^T
// *
// >>>>>                                            */
//return_t multiplyInvQAT(	qpData_t* const qpData,
//						xx_matrix_t* const res,
//						const xx_matrix_t* const cholQ,
//						const xx_matrix_t* const A,
//						x_vector_t* const vecTmp
//						)
//{
//	return multiplyInvMatrixMatrixT(	qpData, (matrix_t*)res, (matrix_t*)cholQ, (matrix_t*)A, (vector_t*)vecTmp, _NX_, _NX_ );
//}
///*<<< END OF multiplyInvQA */


/* ----------------------------------------------
 * Matrix times inverse matrix product res = A * Q^-1
 *
 >>>>>                                            */
/** TODO: check whether this routine is still needed! */
return_t multiplyAInvQ(	qpData_t* const qpData,
						xx_matrix_t* const res,
						const xx_matrix_t* const C,
						const xx_matrix_t* const cholH
						)
{
	int_t ii,jj;

	res->sparsityType = QPDUNES_DENSE;

	/** choose appropriate multiplication routine */
	switch( cholH->sparsityType )
	{
		/* cholH dense */
		case QPDUNES_DENSE		:
		case QPDUNES_SPARSE	:
			qpDUNES_printError(qpData, __FILE__, __LINE__, "multiplyAInvQ is currently not supported for dense primal Hessians");
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;

		/* cholH diagonal */
		case QPDUNES_DIAGONAL	:
			/* scale A part of C column-wise */
			for( ii=0; ii<_NX_; ++ii )	{
				for( jj=0; jj<_NX_; ++jj ) {
					#ifdef __DEBUG__
					if ( fabs( cholH->data[ii] ) >= qpData->options.QPDUNES_ZERO * fabs( accC(ii,jj) ) ) {
					#endif
						/* cholH is the actual matrix in diagonal case */
						res->data[ii*_NX_+jj] = accC(ii,jj) / cholH->data[jj];
					#ifdef __DEBUG__
					}
					else {
						qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in multiplyAInvQ. Rank-deficient Hessian?" );
						return QPDUNES_ERR_DIVISION_BY_ZERO;
					}
					#endif
				}
			}
			break;

		/* cholH identity */
		case QPDUNES_IDENTITY	:
			/* copy A block */
			for( ii=0; ii < _NX_; ++ii ) {
				for( jj=0; jj < _NX_; ++jj ) {
					res->data[ii*_NX_+jj] = accC(ii,jj);
				}
			}
			break;

		default				:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Unknown sparsity type of first matrix argument" );
			break;
	}

	return QPDUNES_OK;
}
/*<<< END OF multiplyAInvQ */


/* ----------------------------------------------
 * Inverse matrix times identity matrix product res = Q^-1 * I
 * 
 >>>>>                                            */
return_t getInvQ(	qpData_t* const qpData,
					xx_matrix_t* const res,
					const vv_matrix_t* const cholH,
					int_t nV
					)
{
	switch( cholH->sparsityType )
	{
		/* cholM1 dense */
		case QPDUNES_DENSE		:
		case QPDUNES_SPARSE	:
			qpDUNES_printError(qpData, __FILE__, __LINE__, "getInvQ not supported for dense primal Hessians.");
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;

		/* cholM1 diagonal */
		case QPDUNES_DIAGONAL	:
			res->sparsityType = QPDUNES_DIAGONAL;
			return backsolveMatrixDiagonalIdentity( qpData, res->data, cholH->data, nV );	/* cholH is just save in first line; first _NX_ elements are Q part */

		/* cholM1 identity */
		case QPDUNES_IDENTITY	:
			res->sparsityType = QPDUNES_IDENTITY;
			return QPDUNES_OK;

		default				:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Unknown sparsity type of primal Hessian" );
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
}
/*<<< END OF getInvQ */


/* ----------------------------------------------
 * Compute a Cholesky factorization of Hessian H
 *
 >>>>>>                                           */
return_t factorizeH( 	qpData_t* const qpData,
						vv_matrix_t* const cholH,
						const vv_matrix_t* const H,
						int_t nV
						)
{
	return factorizePosDefMatrix( qpData, (matrix_t*)cholH, (matrix_t*)H, nV );
}
/*<<< END OF factorizeH */


/* ----------------------------------------------
 * ...
 * 
 * >>>>>>                                           */
return_t addCInvHCT(	qpData_t* const qpData,
						xx_matrix_t* const res,
						const vv_matrix_t* const cholH,
						const xz_matrix_t* const C,
						const d2_vector_t* const y,
						xx_matrix_t* const xxMatTmp,
						ux_matrix_t* const uxMatTmp,
						zx_matrix_t* const zxMatTmp
						)
{
	/* TODO: summarize to one function */
	return addMultiplyMatrixInvMatrixMatrixT(qpData, res, cholH, C, (y == 0 ? 0 : y->data),
			zxMatTmp, &(qpData->xVecTmp), _NX_, _NZ_);

	return QPDUNES_OK;
}
/*<<< END OF addCInvHC */


/* ----------------------------------------------
 * ...
 *
 > >>>>                    *                        */
/* TODO: check if really needed */
return_t addScaledLambdaStep(	qpData_t* const qpData,
								xn_vector_t* const res,
								real_t scalingFactor,
								const xn_vector_t* const deltaLambda 	)
{
	return addScaledVector( (vector_t*)res, scalingFactor, (vector_t*)deltaLambda, _NI_ * _NX_ );
}
/*<<< END OF addScaledLambdaStep */


/* ----------------------------------------------
 * ...
 * 
 >>>>>                                            */
return_t copyScaleVector(	vector_t* const res,
							real_t scalingFactor,
							const vector_t* const vec,
							int_t len
							)
{
	int_t ii;

	for( ii = 0; ii < len; ++ii ) {
		res->data[ii] = scalingFactor * vec->data[ii];
	}

	return QPDUNES_OK;
}
/*<<< END OF copyScaleVector */


/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t scaleVector(	vector_t* const res,
						real_t scalingFactor,
						int_t len
						)
{
	int_t ii;
	
	for( ii = 0; ii < len; ++ii ) {
		res->data[ii] = scalingFactor * res->data[ii];
	}
	
	return QPDUNES_OK;
}
/*<<< END OF scaleVector */


/* ----------------------------------------------
 * ...
 * 
 > >>>>                    *                        */
return_t addScaledVector(	vector_t* const res,
							real_t scalingFactor,
							const vector_t* const update,
							int_t len
							)
{
	int_t ii;
	
	for( ii = 0; ii < len; ++ii ) {
		res->data[ii] += scalingFactor * update->data[ii];
	}
	
	return QPDUNES_OK;
}
/*<<< END OF addScaledLambdaStep */


/* ----------------------------------------------
 * res = x + a*y
 * 
 >>>>>                                            */
return_t addVectorScaledVector(	vector_t* const res,
								const vector_t* const x,
								real_t scalingFactor,
								const vector_t* const y,
								int_t len
								)
{
	int_t ii;

	for( ii = 0; ii < len; ++ii ) {
		res->data[ii] = x->data[ii] + scalingFactor * y->data[ii];
	}

	return QPDUNES_OK;
}
/*<<< END OF addVectors */


/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t addVectors(	vector_t* const res,
						const vector_t* const x,
						const vector_t* const y,
						int_t len
						)
{
	int_t ii;

	for( ii = 0; ii < len; ++ii ) {
		res->data[ii] = x->data[ii] + y->data[ii];
	}

	return QPDUNES_OK;
}
/*<<< END OF addVectors */


/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t addToVector(	vector_t* const res,
						const vector_t* const update,
						int_t len
						)
{
	int_t ii;
	
	for( ii = 0; ii < len; ++ii ) {
		res->data[ii] += update->data[ii];
	}
	
	return QPDUNES_OK;
}
/*<<< END OF addToVector */


/* ----------------------------------------------
 * ...
 * 
 >>>>>                                            */
return_t subtractVectors(	vector_t* const res,
							const vector_t* const x,
							const vector_t* const y,
							int_t len
							)
{
	int_t ii;

	for( ii = 0; ii < len; ++ii ) {
		res->data[ii] = x->data[ii] - y->data[ii];
	}

	return QPDUNES_OK;
}
/*<<< END OF subtractVectors */


/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t subtractFromVector(	vector_t* const res,
							const vector_t* const update,
							int_t len
							)
{
	int_t ii;
	
	for( ii = 0; ii < len; ++ii ) {
		res->data[ii] -= update->data[ii];
	}
	
	return QPDUNES_OK;
}
/*<<< END OF subtractFromVector */


/* ----------------------------------------------
 * ...
 * 
 >>>>>                                            */
return_t negateVector(	vector_t* const res,
						int_t len
						)
{
	int_t ii;
	
	for( ii = 0; ii < len; ++ii ) {
		res->data[ii] = -res->data[ii];
	}
	
	return QPDUNES_OK;
}
/*<<< END OF negateVector */


/* ----------------------------------------------
 * ...
 * 
 >>>>>                                            */
return_t addMatrix(	matrix_t* const res,
					const matrix_t* const update,
					int_t dim0,
					int_t dim1
					)
{
	int_t ii, jj;
	
	switch ( res->sparsityType )
	{
		/* res matrix dense */
		case QPDUNES_DENSE		:
			/* resulting matrix is again dense => keep res->sparsityType */
			switch( update->sparsityType )
			{
				case QPDUNES_DENSE		:
					for( ii = 0; ii < dim0*dim1; ++ii ) {
						res->data[ii] += update->data[ii];
					}
					return QPDUNES_OK;
				case QPDUNES_SPARSE		:
					for( ii = 0; ii < dim0*dim1; ++ii ) {
						res->data[ii] += update->data[ii];
					}
					return QPDUNES_OK;
				case QPDUNES_DIAGONAL	:
					for( ii = 0; ii < dim0; ++ii ) {
						res->data[ii*dim0+ii] += update->data[ii];
					}
					return QPDUNES_OK;
				case QPDUNES_IDENTITY	:
					for( ii = 0; ii < dim0; ++ii ) {
						res->data[ii*dim0+ii] += 1.;
					}
					return QPDUNES_OK;
				default			:
					return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
			}
		
		/* res matrix sparse */
		case QPDUNES_SPARSE		:
			switch( update->sparsityType )
			{
				case QPDUNES_DENSE		:
					for( ii = 0; ii < dim0*dim1; ++ii ) {
						res->data[ii] += update->data[ii];
					}
					res->sparsityType = QPDUNES_DENSE;
					return QPDUNES_OK;
				case QPDUNES_SPARSE		:
					for( ii = 0; ii < dim0*dim1; ++ii ) {
						res->data[ii] += update->data[ii];
					}
					/* resulting matrix is again sparse => keep res->sparsityType */
					return QPDUNES_OK;
				case QPDUNES_DIAGONAL	:
					for( ii = 0; ii < dim0; ++ii ) {
						res->data[ii*dim0+ii] += update->data[ii];
					}
					return QPDUNES_OK;
				case QPDUNES_IDENTITY	:
					for( ii = 0; ii < dim0; ++ii ) {
						res->data[ii*dim0+ii] += 1.;
					}
					return QPDUNES_OK;
				default			:
					return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
			}
		
		/* res matrix diagonal */
		case QPDUNES_DIAGONAL	:
			switch( update->sparsityType )
			{
				case QPDUNES_DENSE		:
					/* make res dense */
					res->data[0] += update->data[0];
					/* rest of first row: */
					for( jj = 1; jj < dim0; ++jj ) {
						res->data[jj*dim0+jj] = res->data[jj];
						res->data[jj] = update->data[jj];
					}
					/* following rows */
					for( ii = 1; ii < dim0; ++ii ) {
						/* addition before diagonal */
						for( jj=0; jj<ii; ++jj ) {
							res->data[ii*dim0+jj] = update->data[ii*dim0+jj];
						}
						/* diagonal */
						res->data[ii*dim0+ii] += update->data[ii*dim0+ii];
						/* addition after diagonal */
						for( jj=ii+1; jj<dim0; ++jj ) {
							res->data[ii*dim0+jj] = update->data[ii*dim0+jj];
						}
					}
					res->sparsityType = QPDUNES_DENSE;
					return QPDUNES_OK;
				
				case QPDUNES_SPARSE		:
					/* make res dense */
					res->data[0] += update->data[0];
					/* rest of first row: */
					for( jj = 1; jj < dim0; ++jj ) {
						res->data[jj*dim0+jj] = res->data[jj];
						res->data[jj] = update->data[jj];
					}
					/* following rows */
					for( ii = 1; ii < dim0; ++ii ) {
						/* addition before diagonal */
						for( jj=0; jj<ii; ++jj ) {
							res->data[ii*dim0+jj] = update->data[ii*dim0+jj];
						}
						/* diagonal */
						res->data[ii*dim0+ii] += update->data[ii*dim0+ii];
						/* addition after diagonal */
						for( jj=ii+1; jj<dim0; ++jj ) {
							res->data[ii*dim0+jj] = update->data[ii*dim0+jj];
						}
					}
					res->sparsityType = QPDUNES_SPARSE;
					return QPDUNES_OK;
				
				case QPDUNES_DIAGONAL	:
					for( jj = 0; jj < dim0; ++jj ) {
						res->data[jj] += update->data[jj];
					}
					/* resulting matrix is again diagonal => keep res->sparsityType */
					return QPDUNES_OK;
				
				case QPDUNES_IDENTITY	:
					for( jj = 0; jj < dim0; ++jj ) {
						res->data[jj] += 1.;
					}
					return QPDUNES_OK;
				
				default			:
					return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
			}
			
		/* res matrix identity */
		case QPDUNES_IDENTITY	:
			switch( update->sparsityType )
			{
				case QPDUNES_DENSE		:
					for( ii = 0; ii < dim0; ++ii ) {
						for( jj = 0; jj < dim1; ++jj ) {
							if (jj == ii) {
								res->data[ii*dim1+jj] = update->data[ii*dim1+jj] + 1.;
							}
							else {
								res->data[ii*dim1+jj] = update->data[ii*dim1+jj];
							}
						}
					}
					res->sparsityType = QPDUNES_DENSE;
					return QPDUNES_OK;
				
				case QPDUNES_SPARSE		:
					for( ii = 0; ii < dim0; ++ii ) {
						for( jj = 0; jj < dim1; ++jj ) {
							if (jj == ii) {
								res->data[ii*dim1+jj] = update->data[ii*dim1+jj] + 1.;
							}
							else {
								res->data[ii*dim1+jj] = update->data[ii*dim1+jj];
							}
						}
					}
					res->sparsityType = QPDUNES_SPARSE;
					return QPDUNES_OK;
				
				case QPDUNES_DIAGONAL	:
					res->sparsityType = QPDUNES_DIAGONAL;
					for( jj = 0; jj < dim0; ++jj ) {
						res->data[jj] = update->data[jj] + 1.;
					}
					return QPDUNES_OK;
				
				case QPDUNES_IDENTITY	:
					res->sparsityType = QPDUNES_DIAGONAL;
					for( jj = 0; jj < dim0; ++jj ) {
						res->data[jj] = 2.;
					}
					return QPDUNES_OK;
					
				default			:
					return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
			}
		
		default			:
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
}
/*<<< END OF addMatrix */


/* ----------------------------------------------
 * ...
 * 
 > >>>>>                           *                */
real_t newtonGradientNorm(	const qpData_t* const qpData,
							const xn_vector_t* const vec
)	/* TODO: check whether this function is really needed! */
{
	return vectorNorm( (vector_t*)vec, _NI_ * _NX_ );
}
/*<<< END OF newtonGradientNorm */



/* ----------------------------------------------
 * Backsolve for a dense L
 * compute res for L*res = b
 * and L^T*res = b
 * 
#>>>>                                             */
return_t backsolveDenseL(	qpData_t* const qpData,
							real_t* const res, 
							const real_t* const L, 
							const real_t* const b, 
							boolean_t transposed,
							int_t n
							)
{
	int_t ii, jj;
	real_t sum;	
	
	/* Solve La = b, where L might be transposed. */
	if ( transposed == QPDUNES_FALSE )
	{
		/* solve L*a = b */
		for( ii=0; ii<n; ++ii )
		{
			sum = b[ii];
			for( jj=0; jj<ii; ++jj ) {
				sum -= accL(ii,jj,n) * res[jj];
			}
			
			if ( fabs( accL(ii,ii,n) ) >= qpData->options.QPDUNES_ZERO * fabs( sum ) ) {
				res[ii] = sum / accL(ii,ii,n);
			}
			else {
				qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveDenseL. Rank-deficient Matrix?" );
				return QPDUNES_ERR_DIVISION_BY_ZERO;
			}
		}
	}
	else
	{
		/* solve L^Ta = b */
		for( ii=(n-1); ii>=0; --ii )
		{
			sum = b[ii];
			for( jj=(ii+1); jj<n; ++jj ) {
				sum -= accL(jj,ii,n) * res[jj];
			}
			
			if ( fabs( accL(ii,ii,n) ) >= qpData->options.QPDUNES_ZERO * fabs( sum ) ) {
				res[ii] = sum / accL(ii,ii,n);
			}
			else {
				qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveDenseL. Rank-deficient Matrix?" );
				return QPDUNES_ERR_DIVISION_BY_ZERO;
			}
		}
	}
	
	return QPDUNES_OK;
}
/*<<<< END OF backsolveDenseL */


/* ----------------------------------------------
 * Backsolve for a diagonal M
 * compute res for M*res = b
 * 
#>>>>                                             */
return_t backsolveDiagonal(	qpData_t* const qpData,
							real_t* const res, 
							const real_t* const M, 
							const real_t* const b, 
							int_t n
							)
{
	int_t ii;
	
	/* Solve M*res = b */
	for( ii=0; ii<n; ++ii )
	{
		#ifdef __DEBUG__
		if ( fabs( accM(0,ii,n) ) >= qpData->options.QPDUNES_ZERO * fabs( b[ii] ) ) {
		#endif
			res[ii] = b[ii] / accM(0,ii,n);
		#ifdef __DEBUG__
		}
		else {
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveDiagonal. Rank-deficient Matrix?" );
			return QPDUNES_ERR_DIVISION_BY_ZERO;
		}
		#endif
	}
	
	return QPDUNES_OK;
}


/* ----------------------------------------------
 * Matrix backsolve for L, M both dense
 * compute res for L*res = M
 * 
 >>>>>                                            */
/* TODO: check whether still needed */
return_t backsolveMatrixDenseDenseL( qpData_t* const qpData,
									 real_t* const res,
									 const real_t* const L,
									 const real_t* const M,
									 real_t* const sums,			/**< memory for saving intermediate results (for speedup) */
									boolean_t transposed,
									int_t dim0,
									int_t dim1 )
{
	int_t ii, jj, kk;
	
	/* Solve L*A = B, where L might be transposed. */
	if ( transposed == QPDUNES_FALSE )
	{
		/* solve L*A = B */
		for( ii=0; ii<dim0; ++ii )
		{
			for( kk=0; kk<dim1; ++kk ) {
				sums[kk] = accM(ii,kk,dim1);
			}
			for( jj=0; jj<ii; ++jj ) {
				for( kk=0; kk<dim1; ++kk ) {
					sums[kk] -= accL(ii,jj,dim0) * res[jj * dim1 + kk];
				}
			}
			
			for( kk=0; kk<dim1; ++kk ) 
			{
				if ( fabs( accL(ii,ii,dim0) ) >= qpData->options.QPDUNES_ZERO * fabs( sums[kk] ) ) {
					res[ii * dim1 + kk] = sums[kk] / accL(ii,ii,dim0);
				}
				else {
					qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveMatrixDenseDenseL. Rank-deficient Matrix?" );
					return QPDUNES_ERR_DIVISION_BY_ZERO;
				}
			}
		}
	}
	else
	{
		/* solve L^T*A = B */
		for( ii=(dim0-1); ii>=0; --ii )
		{
			for( kk=0; kk<dim1; ++kk ) {
				sums[kk] = accM(ii,kk,dim1);
			}
			for( jj=(ii+1); jj<dim0; ++jj ) {
				for( kk=0; kk<dim1; ++kk ) {
					sums[kk] -= accL(jj,ii,dim0) * res[jj * dim1 + kk];
				}
			}
			
			for( kk=0; kk<dim1; ++kk ) 
			{
				if ( fabs( accL(ii,ii,dim0) ) >= qpData->options.QPDUNES_ZERO * fabs( sums[kk] ) ) {
					res[ii * dim1 + kk] = sums[kk] / accL(ii,ii,dim0);
				}
				else {
					qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveMatrixDenseDenseL. Rank-deficient Matrix?" );
					return QPDUNES_ERR_DIVISION_BY_ZERO;
				}
			}
		}
	}
	
	return QPDUNES_OK;
}
/*<<< END OF backsolveMatrixDenseDenseL */


/* ----------------------------------------------
 * Matrix backsolve for L, M^T both dense
 * compute res for L*res = M^T
 *
 >>>>>                                            */
return_t backsolveMatrixDenseDenseTL( qpData_t* const qpData,
									 real_t* const res,
									 const real_t* const L,
									 const real_t* const M,
									 real_t* const sums,			/**< memory for saving intermediate results (for speedup) */
									boolean_t transposed,
									int_t dim0,			/* leading dimension of M */
									int_t dim1 			/* secondary dimension of M */
									)
{
	int_t ii, jj, kk;

	/* Solve L*A = M^T, where L might be transposed. */
	if ( transposed == QPDUNES_FALSE )
	{
		/* solve L*A = M^T */
		for( ii=0; ii<dim0; ++ii )
		{
			/* todo: make more efficient for transposed access by not using sums */
			for( kk=0; kk<dim1; ++kk ) {
				sums[kk] = accMT(ii,kk,dim0);
			}
			for( jj=0; jj<ii; ++jj ) {
				for( kk=0; kk<dim1; ++kk ) {
					sums[kk] -= accL(ii,jj,dim0) * res[jj * dim1 + kk];
				}
			}

			for( kk=0; kk<dim1; ++kk )
			{
				#ifdef __DEBUG__
				if ( fabs( accL(ii,ii,dim0) ) >= qpData->options.QPDUNES_ZERO * fabs( sums[kk] ) ) {
				#endif
					res[ii * dim1 + kk] = sums[kk] / accL(ii,ii,dim0);
				#ifdef __DEBUG__
				}
				else {
					qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveMatrixDenseDenseL. Rank-deficient Matrix?" );
					return QPDUNES_ERR_DIVISION_BY_ZERO;
				}
				#endif
			}
		}
	}
	else
	{
		/* solve L^T*A = M^T */
		for( ii=(dim0-1); ii>=0; --ii )
		{
			for( kk=0; kk<dim1; ++kk ) {
				sums[kk] = accMT(ii,kk,dim0);
			}
			for( jj=(ii+1); jj<dim0; ++jj ) {
				for( kk=0; kk<dim1; ++kk ) {
					sums[kk] -= accL(jj,ii,dim0) * res[jj * dim1 + kk];
				}
			}

			for( kk=0; kk<dim1; ++kk )
			{
				#ifdef __DEBUG__
				if ( fabs( accL(ii,ii,dim0) ) >= qpData->options.QPDUNES_ZERO * fabs( sums[kk] ) ) {
				#endif
					res[ii * dim1 + kk] = sums[kk] / accL(ii,ii,dim0);
				#ifdef __DEBUG__
				}
				else {
					qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveMatrixDenseDenseL. Rank-deficient Matrix?" );
					return QPDUNES_ERR_DIVISION_BY_ZERO;
				}
				#endif
			}
		}
	}

	return QPDUNES_OK;
}
/*<<< END OF backsolveMatrixDenseDenseTL */


/* ----------------------------------------------
 * Matrix backsolve for L dense, M identity
 * compute res for L*res = M
 * 
 >>>>>                                            */
return_t backsolveMatrixDenseIdentityL( qpData_t* const qpData,
										real_t* const res,
										const real_t* const L,
										real_t* const sums,			/**< memory for saving intermediate results (for speedup) */
										int_t dim0 )
{
	int_t ii, jj, kk;

	/* solve L*A = I */
	for( ii=0; ii<dim0; ++ii )
	{
		for( kk=0; kk<dim0; ++kk ) {
			sums[kk] = 0.;
		}
		sums[ii] = 1.;
		
		for( jj=0; jj<ii; ++jj ) {
			for( kk=0; kk<dim0; ++kk ) {
				sums[kk] -= accL(ii,jj,dim0) * res[jj * dim0 + kk];
			}
		}
		
		for( kk=0; kk<dim0; ++kk ) 
		{
			if ( fabs( accL(ii,ii,dim0) ) >= qpData->options.QPDUNES_ZERO * fabs( sums[kk] ) ) {
				res[ii * dim0 + kk] = sums[kk] / accL(ii,ii,dim0);
			}
			else {
				qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveMatrixDenseIdentityL. Rank-deficient Matrix?" );
				return QPDUNES_ERR_DIVISION_BY_ZERO;
			}
		}
	}
	
	return QPDUNES_OK;
}
/*<<< END OF backsolveMatrixDenseIdentityL */


/* ----------------------------------------------
 * Matrix backsolve for L diagonal and M dense
 * compute res for L*res = M
 * 
 >>>>>                                            */
return_t backsolveMatrixDiagonalDense( qpData_t* const qpData,
									   real_t* const res,
									   const real_t* const M1,
									   const real_t* const M2,
									int_t dim0,
									int_t dim1 )
{
	int_t ii, jj;
	
	for( ii=0; ii<dim0; ++ii )	{
		for( jj=0; jj<dim1; ++jj ) {
			if ( fabs( M1[ii] ) >= qpData->options.QPDUNES_ZERO * fabs( M2[ii*dim1+jj] ) ) {
				/* M1 is the actual matrix in diagonal case */ 
				res[ii * dim1 + jj] = M2[ii * dim1 + jj] / M1[ii];
			}
			else {
				qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveMatrixDiagonalDense. Rank-deficient Matrix?" );
				return QPDUNES_ERR_DIVISION_BY_ZERO;
			}
		}
	}
	
	return QPDUNES_OK;
}
/*<<< END OF backsolveMatrixDiagonalDense */


/* ----------------------------------------------
 * Matrix backsolve for L diagonal and M^T dense
 * compute res for L*res = M^T
 *
 >>>>>                                            */
return_t backsolveMatrixDiagonalDenseT( qpData_t* const qpData,
									   real_t* const res,
									   const real_t* const L,
									   const real_t* const M,		/**< square matrix */
										int_t dim					/**< leading and secondary dimension of M */
										)
{
	int_t ii, jj;

	for( ii=0; ii<dim; ++ii )	{
		for( jj=0; jj<dim; ++jj ) {
			#ifdef __DEBUG__
			if ( fabs( L[ii] ) >= qpData->options.QPDUNES_ZERO * fabs( accM(ii,jj,dim) ) ) {
			#endif
				/* L is the actual matrix in diagonal case */
				res[ii * dim + jj] = accMT(ii,jj,dim) / L[ii];
			#ifdef __DEBUG__
			}
			else {
				qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveMatrixDiagonalDenseT. Rank-deficient Matrix?" );
				return QPDUNES_ERR_DIVISION_BY_ZERO;
			}
			#endif
		}
	}

	return QPDUNES_OK;
}
/*<<< END OF backsolveMatrixDiagonalDenseT */


/* ----------------------------------------------
 * Matrix backsolve for M1, M2 both diagonal
 * compute res for M1*res = M2
 * 
 >>>>>                                            */
return_t backsolveMatrixDiagonalDiagonal( qpData_t* const qpData,
										  real_t* const res,
										  const real_t* const M1,
										  const real_t* const M2,
										 int_t dim0 )
{
	int_t ii;
	
	/* backsolve on diagonal matrix: res for M1*res = M2 */
	for( ii=0; ii<dim0; ++ii )
	{
		if ( fabs( M1[ii] ) >= qpData->options.QPDUNES_ZERO * fabs( M2[ii] ) ) {
			/* M1 is the actual matrix in diagonal case */ 
			res[ii] = M2[ii] / M1[ii];
		}
		else {
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveMatrixDiagonalDiagonal. Rank-deficient Matrix?" );
			return QPDUNES_ERR_DIVISION_BY_ZERO;
		}
	}
	
	return QPDUNES_OK;
}
/*<<< END OF backsolveMatrixDiagonalDiagonal */


/* ----------------------------------------------
 * Matrix backsolve for M1 diagonal, M2 identity
 * compute res for M1*res = I
 * 
 >>>>>                                            */
return_t backsolveMatrixDiagonalIdentity( qpData_t* const qpData,
										  real_t* const res,
										  const real_t* const M1,
										 int_t dim0 )
{
	int_t ii;
	
	/* backsolve on diagonal matrix: res for M1*res = I */
	for( ii=0; ii<dim0; ++ii )
	{
		if ( fabs( M1[ii] ) >= qpData->options.QPDUNES_ZERO ) {
			/* M1 is the actual matrix in diagonal case */ 
			res[ii] = 1. / M1[ii];
		}
		else {
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveMatrixDiagonalIdentity. Rank-deficient Matrix?" );
			return QPDUNES_ERR_DIVISION_BY_ZERO;
		}
	}
	
	return QPDUNES_OK;
}
/*<<< END OF backsolveMatrixDiagonalDiagonal */


/* ----------------------------------------------
 * Matrix backsolve for L, M both dense
 * compute res for L*res = M^T
 * 
 >>>>>                                            */
return_t backsolveMatrixTDenseDenseL( qpData_t* const qpData,
									 real_t* const res,
									 const real_t* const L,
									 const real_t* const M,			/**< untransposed M */
									 real_t* const sums,			/**< memory for saving intermediate results (for speedup) */
									 boolean_t transposedL,
									 int_t dim0,					/**< dimensions of M */
									 int_t dim1 )
{
	int_t ii, jj, kk;
	
	/* Solve L*A = B^T, where L might be transposed. */
	if ( transposedL == QPDUNES_FALSE )
	{
		/* solve L*A = B^T */
		for( ii=0; ii<dim1; ++ii )	/* go by rows */
		{
			for( kk=0; kk<dim0; ++kk ) {
				sums[kk] = accM(kk,ii,dim0);
			}
			for( jj=0; jj<ii; ++jj ) {
				for( kk=0; kk<dim0; ++kk ) {
					sums[kk] -= accL(ii,jj,dim1) * res[jj * dim0 + kk];
				}
			}
			
			for( kk=0; kk<dim0; ++kk ) 
			{
				if ( fabs( accL(ii,ii,dim1) ) >= qpData->options.QPDUNES_ZERO * fabs( sums[kk] ) ) {
					res[ii * dim0 + kk] = sums[kk] / accL(ii,ii,dim1);
				}
				else {
					qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveMatrixTDenseDenseL. Rank-deficient Matrix?" );
					return QPDUNES_ERR_DIVISION_BY_ZERO;
				}
			}
		}
	}
	else
	{
		/* solve L^T*A = B^T */
		for( ii=(dim1-1); ii>=0; --ii )		/* go by rows, bottom-up */
		{
			for( kk=0; kk<dim0; ++kk ) {
				sums[kk] = accM(kk,ii,dim0);
			}
			for( jj=(ii+1); jj<dim1; ++jj ) {
				for( kk=0; kk<dim0; ++kk ) {
					sums[kk] -= accL(jj,ii,dim1) * res[jj * dim0 + kk];
				}
			}
			
			for( kk=0; kk<dim0; ++kk ) 
			{
				if ( fabs( accL(ii,ii,dim1) ) >= qpData->options.QPDUNES_ZERO * fabs( sums[kk] ) ) {
					res[ii * dim0 + kk] = sums[kk] / accL(ii,ii,dim1);
				}
				else {
					qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveMatrixTDenseDenseL. Rank-deficient Matrix?" );
					return QPDUNES_ERR_DIVISION_BY_ZERO;
				}
			}
		}
	}
	
	return QPDUNES_OK;
}
/*<<< END OF backsolveMatrixTDenseDenseL */


/* ----------------------------------------------
 * Matrix backsolve for L diagonal and M dense
 * compute res for L*res = M^T
 * 
 > >>>>                                      *      */
return_t backsolveMatrixTDiagonalDense( qpData_t* const qpData,
									   real_t* const res,
									   const real_t* const M1,
									   const real_t* const M2,	/**< untransposed M2 */
									   int_t dim0,				/**< dimensions of M2 */
									   int_t dim1 )
{
	int_t ii, jj;
	
	for( ii=0; ii<dim1; ++ii )	{
		for( jj=0; jj<dim0; ++jj ) {
			if ( fabs( M1[ii] ) >= qpData->options.QPDUNES_ZERO * fabs( M2[jj*dim1+ii] ) ) {
				/* M1 is the actual matrix in diagonal case; M2 is untransposed */ 
				res[ii * dim0 + jj] = M2[jj * dim1 + ii] / M1[ii];
			}
			else {
				qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveMatrixTDiagonalDenseL. Rank-deficient Matrix?" );
				return QPDUNES_ERR_DIVISION_BY_ZERO;
			}
		}
	}
	
	return QPDUNES_OK;
}
/*<<< END OF backsolveMatrixTDiagonalDense */


return_t backsolveRT_ZTET(	qpData_t* const qpData,
							zx_matrix_t* const res,
							const zz_matrix_t* const RT,
							const zz_matrix_t* const ZT,
							x_vector_t* const sums,
							int_t dim0, /* number of physical rows and columns in  RT (storage) = number of columns in ZT */
							int_t dim1 	/* number of defined rows in ZT = number of defined rows and columns in RT */
							)
{
	int_t ii, jj, kk;

	int_t dim1SkipIdx = _NX_; /* stopping after the first _NX_ columns of ZT is equivalent to multiplication ZT*ET */

	/* solve RT*res = ZT */
	for (ii = 0; ii < dim1; ++ii) {
		for (kk = 0; kk < dim1SkipIdx; ++kk) {
			sums->data[kk] = ZT->data[ii * dim0 + kk];
		}
		for (jj = 0; jj < ii; ++jj) {
			for (kk = 0; kk < dim1SkipIdx; ++kk) {
				sums->data[kk] -= RT->data[ii * dim0 + jj] * res->data[jj * dim1SkipIdx + kk];
			}
		}

		for (kk = 0; kk < dim1SkipIdx; ++kk) {
//			res->data[ii * dim1SkipIdx + kk] = sums->data[kk] / RT->data[ii * dim0 + jj];
			res->data[ii * dim1SkipIdx + kk] = sums->data[kk] / RT->data[ii * dim0 + ii];
		}
	}

	return QPDUNES_OK;
}
/*<<< END OF backsolveRT_ZTET */


/* TODO: merge with ZTET backsolve above!!! */
return_t backsolveRT_ZTCT(	qpData_t* const qpData,
							zx_matrix_t* const res,
							const zz_matrix_t* const RT,
							const zz_matrix_t* const ZTCT,
							x_vector_t* const sums,
							int_t dim0, /* number of physical rows and columns in RT (storage) */
							int_t dim1 	/* number of (well-defined) rows in ZTCT (same as ZT) */
							)
{
	int_t ii, jj, kk;

	int_t dim2 = _NX_; /* number of columns in ZTCT */

	/* solve RT*res = ZT */
	for (ii = 0; ii < dim1; ++ii) {
		for (kk = 0; kk < dim2; ++kk) {
			sums->data[kk] = ZTCT->data[ii * dim2 + kk];
		}
		for (jj = 0; jj < ii; ++jj) {
			for (kk = 0; kk < dim2; ++kk) {
				sums->data[kk] -= RT->data[ii * dim0 + jj] * res->data[jj * dim2 + kk];
			}
		}

		for (kk = 0; kk < dim2; ++kk) {
//			res->data[ii * dim2 + kk] = sums->data[kk] / RT->data[ii * dim0 + jj];
			res->data[ii * dim2 + kk] = sums->data[kk] / RT->data[ii * dim0 + ii];
		}
	}

	return QPDUNES_OK;
}
/*<<< END OF backsolveRT_ZTCT */



/* -------------------------------------------------------------
 * G E N E R I C    M A T R I X - V E C T O R    P R O D U C T S
 * ------------------------------------------------------------- */


/* ----------------------------------------------
 * Generic matrix-vector product b = M*x
 * 
 >>>>>                                            */
return_t multiplyMatrixVector( vector_t* const res,
							   const matrix_t* const M,
							   const vector_t* const x,
							   int_t dim0,
							   int_t dim1		)
{
	/** choose appropriate multiplication routine */
	switch( M->sparsityType )	
	{
		case QPDUNES_DENSE		:
			return multiplyMatrixVectorDense( res->data, M->data, x->data, dim0, dim1 );
		case QPDUNES_SPARSE		:
			return multiplyMatrixVectorSparse( res->data, M->data, x->data, dim0, dim1 );
		case QPDUNES_DIAGONAL	:
			return multiplyMatrixVectorDiagonal( res->data, M->data, x->data, dim0 );
		case QPDUNES_IDENTITY	:
			/* just copy vector */
			return qpDUNES_copyVector( res, x, dim0 );
		default			:
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
}
/*<<< END OF multiplyMatrixVector */


/* ----------------------------------------------
 * Generic vector-matrix-vector product b = x'*M*x
 *  - M has to be square matrix
 *
 >>>>>                                            */
real_t multiplyVectorMatrixVector(	const matrix_t* const M,
							   	   	const vector_t* const x,
							   	   	int_t dim0	)
{
	/** choose appropriate multiplication routine */
	switch( M->sparsityType )
	{
		case QPDUNES_DENSE		:
		case QPDUNES_SPARSE	:
			return multiplyVectorMatrixVectorDense( M->data, x->data, dim0 );

		case QPDUNES_DIAGONAL	:
			return multiplyVectorMatrixVectorDiagonal( M->data, x->data, dim0 );

		case QPDUNES_IDENTITY	:
			/* just square vector */
			return scalarProd( x, x, dim0 );

		default			:
			return -1;
	}
}
/*<<< END OF multiplyVectorMatrixVector */


/* ----------------------------------------------
 * Generic block-diagonal matrix-vector product b = M*x
 * for matrix M consisting of two blocks on diagonal: M1, M2
 * 
 >>>>>                                            */
/* TODO: check if this routine is still needed */
return_t multiplyBlockDiagMatrixVector( vector_t* const res,
										const matrix_t* const M1,
										const matrix_t* const M2,
										const vector_t* const x,
										int_t dimM1,			/**< dimensions of M1 */
										int_t dimM2 			/**< dimensions of M2 */
										)
{
	int_t ii;
	
	/** choose appropriate multiplication routine for M1 */
	switch( M1->sparsityType )	
	{
		case QPDUNES_DENSE		:
			multiplyMatrixVectorDense( res->data, M1->data, x->data, dimM1, dimM1 );
			break;
		case QPDUNES_SPARSE		:
			multiplyMatrixVectorSparse( res->data, M1->data, x->data, dimM1, dimM1 );
			break;
		case QPDUNES_DIAGONAL	:
			multiplyMatrixVectorDiagonal( res->data, M1->data, x->data, dimM1 );
			break;
		case QPDUNES_IDENTITY	:
			/* just copy vector */
			for( ii=0; ii<dimM1; ++ii ) {
				res->data[ii] = x->data[ii];
			}
			break;
		case QPDUNES_ALLZEROS	:
			/* set zero vector */
			for( ii=0; ii<dimM1; ++ii ) {
				res->data[ii] = 0.;
			}
			break;
			
		default			:
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
	
	/** choose appropriate multiplication routine for M2 */
	switch( M2->sparsityType )	
	{
		case QPDUNES_DENSE		:	/* multiply offsetted vector */
			multiplyMatrixVectorDense( &(res->data[dimM1]), M2->data, &(x->data[dimM1]), dimM2, dimM2 );
			break;
		case QPDUNES_SPARSE		:
			multiplyMatrixVectorSparse( &(res->data[dimM1]), M2->data, &(x->data[dimM1]), dimM2, dimM2 );
			break;
		case QPDUNES_DIAGONAL	:
			multiplyMatrixVectorDiagonal( &(res->data[dimM1]), M2->data, &(x->data[dimM1]), dimM2 );
			break;
		case QPDUNES_IDENTITY	:
			/* just copy vector */
			for( ii=0; ii<dimM2; ++ii ) {
				res->data[dimM1+ii] = x->data[dimM1+ii];
			}
			break;
		case QPDUNES_ALLZEROS	:
			/* set zero vector */
			for( ii=0; ii<dimM2; ++ii ) {
				res->data[dimM1+ii] = 0.;
			}
			break;
			
		default			:
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
	
	return QPDUNES_OK;
}
/*<<< END OF multiplyBlockDiagMatrixVector */


/* ----------------------------------------------
 * Generic block-diagonal vector-matrix-vector product a = x'*M*x
 * for matrix M consisting of two blocks on diagonal: M1, M2
 *
 >>>>>                                            */
real_t multiplyBlockDiagVectorMatrixVector( const matrix_t* const M1,
											const matrix_t* const M2,
											const vector_t* const x,
											int_t dimM1,			/**< dimensions of M1 */
											int_t dimM2 			/**< dimensions of M2 */
											)
{
	int_t ii;

	real_t result = 0.;

	/** choose appropriate multiplication routine for M1 */
	switch( M1->sparsityType )
	{
		case QPDUNES_DENSE		:
		case QPDUNES_SPARSE	:
			result += multiplyVectorMatrixVectorDense( M1->data, x->data, dimM1 );
			break;
		case QPDUNES_DIAGONAL	:
			result += multiplyVectorMatrixVectorDiagonal( M1->data, x->data, dimM1 );
			break;
		case QPDUNES_IDENTITY	:
			/* just square vector */
			for( ii=0; ii<dimM1; ++ii ) {
				result += x->data[ii] * x->data[ii];
			}
			break;
		case QPDUNES_ALLZEROS	:
		case QPDUNES_MATRIX_UNDEFINED :
			break;	/* add nothing */

		default			:
			return -1e12;
	}

	/** choose appropriate multiplication routine for M2 */
	switch( M2->sparsityType )
	{
		case QPDUNES_DENSE		:	/* multiply offsetted vector */
		case QPDUNES_SPARSE		:
			result += multiplyVectorMatrixVectorDense( M2->data, &(x->data[dimM1]), dimM2 );
			break;
		case QPDUNES_DIAGONAL	:
			result += multiplyVectorMatrixVectorDiagonal( M2->data, &(x->data[dimM1]), dimM2 );
			break;
		case QPDUNES_IDENTITY	:
			/* just square vector */
			for( ii=0; ii<dimM2; ++ii ) {
				result += x->data[dimM1+ii] * x->data[dimM1+ii];
			}
			break;
		case QPDUNES_ALLZEROS	:
		case QPDUNES_MATRIX_UNDEFINED :
			break;	/* add nothing */

		default			:
			return -1e12;
	}

	return result;
}
/*<<< END OF multiplyBlockDiagMatrixVector */


/* ----------------------------------------------
 * Generic transposed matrix-vector product 
 * res = A.T*x
 * 
 >>>>>                              *              */
return_t multiplyMatrixTVector(	vector_t* const res,
								const matrix_t* const M,
								const vector_t* const x,
								int_t dim0,			/* of untransposed matrix */
								int_t dim1		)
{
	/** choose appropriate multiplication routine */
	switch( M->sparsityType )	
	{
		case QPDUNES_DENSE		:
			return multiplyMatrixTVectorDense( res->data, M->data, x->data, dim0, dim1 );
		case QPDUNES_SPARSE		:
			return multiplyMatrixTVectorSparse( res->data, M->data, x->data, dim0, dim1 );
		case QPDUNES_DIAGONAL	:
			return multiplyMatrixVectorDiagonal( res->data, M->data, x->data, dim0 );
		case QPDUNES_IDENTITY	:
			/* just copy vector */
			return qpDUNES_copyVector( res, x, dim0 );
		default			:
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
}
/*<<< END OF multiplyMatrixTVector */


/* ----------------------------------------------
 * Matrix-vector product res = invM*x,
 * using a Cholesky factorization H = L*L^T, where L is a lower triangular matrix.
 * Solve L*L^T * res = x for res by
 * 1) solving L*y = x for y
 * 2) solving L^T*res = y for res
 * 
 >>>>>                                            */
return_t multiplyInvMatrixVector(	qpData_t* const qpData,
									vector_t* const res,
									const matrix_t* const cholH,
									const vector_t* const x,
									int_t dim0						/**< dimension of symmetric matrix */
									)
{
	return_t statusFlag;

	/** choose appropriate multiplication routine */
	switch( cholH->sparsityType )	
	{
		case QPDUNES_DENSE		:
			/* first backsolve: L*y = z */
			statusFlag = backsolveDenseL( qpData, res->data, cholH->data, x->data, QPDUNES_FALSE, dim0 );
			if( statusFlag != QPDUNES_OK ) return statusFlag;
			/* second backsolve: L^T*res = y */
			return backsolveDenseL( qpData, res->data, cholH->data, res->data, QPDUNES_TRUE, dim0 );
			
		case QPDUNES_SPARSE	:
			qpDUNES_printWarning( qpData, __FILE__, __LINE__, "Sparse inverse matrix-vector product not implemented. Using dense multiplication instead." );
			/* first backsolve: L*y = z */
			statusFlag = backsolveDenseL( qpData, res->data, cholH->data, x->data, QPDUNES_FALSE, dim0 );
			if( statusFlag != QPDUNES_OK ) return statusFlag;
			/* second backsolve: L^T*res = y */
			return backsolveDenseL( qpData, res->data, cholH->data, res->data, QPDUNES_TRUE, dim0 );\
			
		case QPDUNES_DIAGONAL	:
			return backsolveDiagonal( qpData, res->data, cholH->data, x->data, dim0 );	/* cholH in this case contains full diagonal matrix (not a factor) */
			
		case QPDUNES_IDENTITY	:
			/* just copy vector */
			return qpDUNES_copyVector( res, x, dim0 );
			
		default				:
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
}
/*<<< END OF multiplyInvMatrixVector */


/* ----------------------------------------------
 * Matrix-vector product res = invM*x,
 * where M is a block diagonal matrix M1, M2
 * using a Cholesky factorization H = L*L^T, where L is a lower triangular matrix.
 * Solve L*L^T * res = x for res by
 * 1) solving L*y = x for y
 * 2) solving L^T*res = y for res
 * 
 >>>>>                                            */
return_t multiplyInvBlockDiagMatrixVector(	qpData_t* const qpData,
									vector_t* const res,
									const matrix_t* const cholM1,
									const matrix_t* const cholM2,
									const vector_t* const x,
									int_t dimM1,					/**< dimensions of M1 */
									int_t dimM2 					/**< dimensions of M2 */
									)
{
	int_t ii;
	return_t statusFlag = QPDUNES_OK;
	
	/** choose appropriate multiplication routine for cholM1 */
	switch( cholM1->sparsityType )	
	{
		case QPDUNES_DENSE		:
			/* first backsolve: L*y = z */
			statusFlag = backsolveDenseL( qpData, res->data, cholM1->data, x->data, QPDUNES_FALSE, dimM1 );
			/* second backsolve: L^T*res = y */
			statusFlag = backsolveDenseL( qpData, res->data, cholM1->data, res->data, QPDUNES_TRUE, dimM1 );
			break;
			
		case QPDUNES_SPARSE	:
			qpDUNES_printWarning( qpData, __FILE__, __LINE__, "Sparse inverse matrix-vector product not implemented. Using dense multiplication instead." );
			/* first backsolve: L*y = z */
			statusFlag = backsolveDenseL( qpData, res->data, cholM1->data, x->data, QPDUNES_FALSE, dimM1 );
			/* second backsolve: L^T*res = y */
			statusFlag = backsolveDenseL( qpData, res->data, cholM1->data, res->data, QPDUNES_TRUE, dimM1 );
			break;
			
		case QPDUNES_DIAGONAL	:
			statusFlag = backsolveDiagonal( qpData, res->data, cholM1->data, x->data, dimM1 );	/* cholM1 in this case contains full diagonal matrix (not only a factor) */
			break;
			
		case QPDUNES_IDENTITY	:
			/* just copy vector */
			statusFlag = qpDUNES_copyVector( res, x, dimM1 );
			break;
			
		default				:
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
	
	if (statusFlag != QPDUNES_OK) {
		return statusFlag;
	}

	
	/** choose appropriate multiplication routine for cholM2 */
	switch( cholM2->sparsityType )	
	{
		case QPDUNES_DENSE		:
			/* first backsolve: L*y = z */
			statusFlag = backsolveDenseL( qpData, &(res->data[dimM1]), cholM2->data, &(x->data[dimM1]), QPDUNES_FALSE, dimM2 );
			/* second backsolve: L^T*res = y */
			statusFlag = backsolveDenseL( qpData, &(res->data[dimM1]), cholM2->data, &(res->data[dimM1]), QPDUNES_TRUE, dimM2 );
			break;
			
		case QPDUNES_SPARSE	:
			qpDUNES_printWarning( qpData, __FILE__, __LINE__, "Sparse inverse matrix-vector product not implemented. Using dense multiplication instead." );
			/* first backsolve: L*y = z */
			statusFlag = backsolveDenseL( qpData, &(res->data[dimM1]), cholM2->data, &(x->data[dimM1]), QPDUNES_FALSE, dimM2 );
			/* second backsolve: L^T*res = y */
			statusFlag = backsolveDenseL( qpData, &(res->data[dimM1]), cholM2->data, &(res->data[dimM1]), QPDUNES_TRUE, dimM2 );
			break;
			
		case QPDUNES_DIAGONAL	:
			statusFlag = backsolveDiagonal( qpData, &(res->data[dimM1]), cholM2->data, &(x->data[dimM1]), dimM2 );	/* cholM2 in this case contains full diagonal matrix (not only a factor) */
			break;
			
		case QPDUNES_IDENTITY	:
			/* just copy vector */
			for( ii=0; ii<dimM2; ++ii ) {
				res->data[dimM1+ii] = x->data[dimM1+ii];
			}
			break;
			
		default				:
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
	
	return statusFlag;
}
/*<<< END OF multiplyInvBlockDiagMatrixVector */


/* ----------------------------------------------
 * Inverse matrix times transposed matrix product
 *
 >>>>>                                            */
return_t multiplyInvMatrixMatrixT(	qpData_t* const qpData,
									matrix_t* const res,
									const matrix_t* const cholM1,
									matrix_t* const M2,		/**< WARNING: M2 might be made dense during this routine */
									vector_t* const vecTmp,
									int_t dim0,				/**< leading dimension of A == secondary dimension of A == leading dimension of M2 */
									int_t dim1				/**< secondary dimension of M2 */
									)
{
	return_t statusFlag = QPDUNES_OK;


	/** choose appropriate multiplication routine */
	switch( cholM1->sparsityType )
	{
		/* cholM1 dense */
		case QPDUNES_DENSE		:
		case QPDUNES_SPARSE	:
			res->sparsityType = QPDUNES_DENSE;
			switch( M2->sparsityType )
			{
				case QPDUNES_DENSE		:
				case QPDUNES_SPARSE	:
				case QPDUNES_DIAGONAL	:
					qpDUNES_makeMatrixDense( M2, dim0, dim1 );
					/* first backsolve: L*Y = M2 */
					backsolveMatrixDenseDenseTL( qpData, res->data, cholM1->data, M2->data, vecTmp->data, QPDUNES_FALSE, dim0, dim1 );
					/* second backsolve: L^T*res = Y */
					backsolveMatrixDenseDenseL( qpData, res->data, cholM1->data, res->data, vecTmp->data, QPDUNES_TRUE, dim0, dim1 );
					return QPDUNES_OK;

				case QPDUNES_IDENTITY	:
					res->sparsityType = QPDUNES_DENSE;
					/* first backsolve: L*Y = M2 */
					statusFlag = backsolveMatrixDenseIdentityL( qpData, res->data, cholM1->data, vecTmp->data, dim0 );
					if( statusFlag != QPDUNES_OK ) return statusFlag;
					/* second backsolve: L^T*res = Y */
					return backsolveMatrixDenseDenseL( qpData, res->data, cholM1->data, res->data, vecTmp->data, QPDUNES_TRUE, dim0, dim1 );
					/* TODO: transpose? */

				default				:
					qpDUNES_printError( qpData, __FILE__, __LINE__, "Unknown sparsity type of second matrix argument" );
					return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
			}


		/* cholM1 diagonal */
		case QPDUNES_DIAGONAL	:
			switch( M2->sparsityType )
			{
				case QPDUNES_DENSE		:
				case QPDUNES_SPARSE		:
					res->sparsityType = QPDUNES_DENSE;
					return backsolveMatrixDiagonalDenseT( qpData, res->data, cholM1->data, M2->data, dim0 );

				case QPDUNES_DIAGONAL	:
					res->sparsityType = QPDUNES_DIAGONAL;
					return backsolveMatrixDiagonalDiagonal( qpData, res->data, cholM1->data, M2->data, dim0 );

				case QPDUNES_IDENTITY	:
					res->sparsityType = QPDUNES_DIAGONAL;
					return backsolveMatrixDiagonalIdentity( qpData, res->data, cholM1->data, dim0 );

				default				:
					qpDUNES_printError( qpData, __FILE__, __LINE__, "Unknown sparsity type of second matrix argument" );
					return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
			}

		/* cholM1 identity */
		case QPDUNES_IDENTITY	:
			switch( M2->sparsityType )
			{
				case QPDUNES_DENSE		:
				case QPDUNES_SPARSE		:
				case QPDUNES_DIAGONAL	:
					/* just transpose M2 matrix */
					return qpDUNES_transposeMatrix( res, M2, dim0, dim1 );

				case QPDUNES_IDENTITY :
					/* copy cholM1, since M2 might be unallocated in this case (getInvQ) */
					res->sparsityType = QPDUNES_IDENTITY;
					return QPDUNES_OK;

				default				:
					qpDUNES_printError( qpData, __FILE__, __LINE__, "Unknown sparsity type of second matrix argument" );
					return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
			}

		default				:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Unknown sparsity type of first matrix argument" );
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
}
/*<<< END OF multiplyInvMatrixMatrixT */



/* ----------------------------------------------
 * Dense generic matrix-vector product b = A*x
 * 
 >>>>>                                            */
return_t multiplyMatrixVectorDense(	real_t* const res,
									const real_t* const M,
									const real_t* const x,
									int_t dim0,
									int_t dim1		)
{
	int_t ii, jj;
	
	for( ii = 0; ii < dim0; ++ii ) {
		res[ii] = 0.;
		for( jj = 0; jj < dim1; ++jj ) {
			res[ii] += accM(ii,jj,dim1) * x[jj];
		}
	}
	
	return QPDUNES_OK;
}
/*<<< END OF multiplyMatrixVectorDense */


/* ----------------------------------------------
 * Dense generic vector-matrix-vector product a = x'*M*x
 *
 >>>>>                                            */
real_t multiplyVectorMatrixVectorDense(	const real_t* const M,
										const real_t* const x,
										int_t dim0 )
{
	int_t ii, jj;
	real_t result = 0.;

	for( ii = 0; ii < dim0; ++ii ) {
		for( jj = 0; jj < dim0; ++jj ) {
			result += x[ii] * accM(ii,jj,dim0) * x[jj];
		}
	}

	return result;
}
/*<<< END OF multiplyVectorMatrixVectorDense */


/* ---------------------------------------------- 
 * Sparse generic matrix-vector product b = A*x
 * WARNING: sparse matrix-vector multiplication 
 * not yet implemented
 * 
 >>>>>>                                           */
return_t multiplyMatrixVectorSparse(	real_t* res,
										const real_t* const M,
										const real_t* const x,
										int_t dim0,
										int_t dim1		)
{
	return multiplyMatrixVectorDense( res, M, x, dim0, dim1 );
}
/*<<< END OF multiplyMatrixVectorSparse */


/* ----------------------------------------------
 * Generic diagonal matrix-vector product b = A*x
 * 
 >>>>>>                                           */
return_t multiplyMatrixVectorDiagonal(	real_t* const res,
										const real_t* const M,
										const real_t* const x,
										int_t dim0	 	)
{
	int_t ii;
	
	/** multiply vector with diagonal matrix saved in first line */
	for( ii = 0; ii < dim0; ++ii ) {
		res[ii] = accM(0,ii,dim0) * x[ii];
	}
	
	return QPDUNES_OK;
}
/*<<< END OF multiplyMatrixVectorDiagonal */


/* ----------------------------------------------
 * Generic diagonal vector-matrix-vector product a = x'*M*x
 *
 >>>>>>                                           */
real_t multiplyVectorMatrixVectorDiagonal(	const real_t* const M,
											const real_t* const x,
											int_t dim0	 	)
{
	int_t jj;
	real_t result = 0.;

	/** multiply vector with diagonal matrix saved in first line */
	for( jj = 0; jj < dim0; ++jj ) {
		result += accM(0,jj,dim0) * x[jj] * x[jj];
	}

	return result;
}
/*<<< END OF multiplyVectorMatrixVectorDiagonal */


/* ----------------------------------------------
 * Dense generic transposed matrix-vector product 
 * res = M.T*x
 * 
 >>>>>                                            */
return_t multiplyMatrixTVectorDense(	real_t* const res,
										const real_t* const M,	/**< untransposed matrix */
										const real_t* const x,
										int_t dim0,					/**< leading dimension of untransposed matrix */
										int_t dim1		)
{
	int_t ii, jj;
	
	/* change multiplication order for more efficient memory access */
	for( jj = 0; jj < dim1; ++jj ) {
		res[jj] = 0.;
	}
	for( ii = 0; ii < dim0; ++ii ) {
		for( jj = 0; jj < dim1; ++jj ) {
			res[jj] += accM(ii,jj,dim1) * x[ii];
		}
	}
	
	return QPDUNES_OK;
}
/*<<< END OF multiplyMatrixVectorDense */


/* ---------------------------------------------- 
 * Sparse generic transposed matrix-vector product 
 * b = A*x
 * WARNING: sparse matrix-vector multiplication 
 * not yet implemented
 * 
 >>>>>>                                           */
return_t multiplyMatrixTVectorSparse(	real_t* res,
										const real_t* const M,
										const real_t* const x,
										int_t dim0,
										int_t dim1		)
{
	return multiplyMatrixTVectorDense( res, M, x, dim0, dim1 );
}
/*<<< END OF multiplyMatrixVectorSparse */


/* ----------------------------------------------
 * Dense generic transposed matrix-matrix product
 * res = M1.T*M2
 *
 >>>>>                                            */
void multiplyMatrixTMatrixDenseDense(	real_t* const res,
										const real_t* const M1,		/**< untransposed matrix */
										const real_t* const M2,
										int_t dim0,					/**< leading dimension of untransposed M1 = leading dimension of M2 */
										int_t dim1,					/**< secondary dimension of untransposed M1 */
										int_t dim2,					/**< secondary dimension of M2 */
										boolean_t addToRes			/**< flag to specify whether to overwrite res, or simply add to it */
										)
{
	int_t ii, jj, kk;

	/* change multiplication order for more efficient memory access */
	if ( addToRes != QPDUNES_TRUE ) {
		for( jj = 0; jj < dim1*dim2; ++jj ) {
			res[jj] = 0.;
		}
	}
	for( ii = 0; ii < dim0; ++ii ) {
		for( jj = 0; jj < dim1; ++jj ) {
			for( kk = 0; kk < dim2; ++kk ) {
				res[jj*dim2+kk] += M1[ii*dim1+jj] * M2[ii*dim2+kk];
			}
		}
	}

	return;
}
/*<<< END OF multiplyMatrixTMatrixDenseDense */


/* ----------------------------------------------
 * Dense generic matrix-transposed matrix product
 * res = M1*M2.T
 *
 >>>>>                                            */
void multiplyMatrixMatrixTDenseDense(	real_t* const res,
										const real_t* const M1,		/**< untransposed matrix */
										const real_t* const M2,
										int_t dim0,					/**< leading dimension of M1 */
										int_t dim1,					/**< secondary dimension of M1 = secondary dimension of untransposed M2 */
										int_t dim2					/**< leading dimension of untransposed M2 */
										)
{
	int_t ii, jj, kk;

	for( ii = 0; ii < dim0; ++ii ) {
		for( jj = 0; jj < dim2; ++jj ) {
			res[ii*dim2+jj] = 0.;
			for( kk = 0; kk < dim1; ++kk ) {
				res[ii*dim2+jj] += M1[ii*dim1+kk] * M2[jj*dim1+kk];	/* transposed access of M2 */
			}
		}
	}

	return;
}
/*<<< END OF multiplyMatrixMatrixTDenseDense */


/* ----------------------------------------------
 *  M2 * M1^-1 * M2.T
 *  result gets added to res, not overwritten
 *
 * >>>>>>                                           */
return_t addMultiplyMatrixInvMatrixMatrixT(	qpData_t* const qpData,
											matrix_t* const res,
											const matrix_t* const cholM1,
											const matrix_t* const M2,
											const real_t* const y, /**< vector containing non-zeros for columns of M2 to be eliminated for unconstrained case */
											matrix_t* const Ztmp, /**< temporary matrix of shape dim1 x dim0 */
											vector_t* const vecTmp,
											int_t dim0, /**< dimensions of M2 */
											int_t dim1
											)
{
	int_t ii, jj, ll;

	/* always assuming M2 is dense */
//	assert( M2->sparsityType == QPDUNES_DENSE );
	/* compute M1^-1/2 * M2.T */
	switch (cholM1->sparsityType) {
	case QPDUNES_DENSE:
	case QPDUNES_SPARSE:
		backsolveMatrixTDenseDenseL(qpData, Ztmp->data, cholM1->data, M2->data,
				vecTmp->data, QPDUNES_FALSE, dim0, dim1);
		break;

	case QPDUNES_DIAGONAL:
		backsolveMatrixTDiagonalDense(qpData, Ztmp->data, cholM1->data,
				M2->data, dim0, dim1); /* computes full inverse times M2! */
		break;

	case QPDUNES_IDENTITY:
		qpDUNES_transposeMatrix(Ztmp, M2, dim0, dim1);
		break;

	default:
		return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}

	qpDUNES_makeMatrixDense(res, dim0, dim0);
	if (cholM1->sparsityType != QPDUNES_DIAGONAL)
	{
		/* compute Z.T * Z as dyadic products */
		for (ll = 0; ll < dim1; ++ll) {
			/* only add columns of variables with inactive bounds */
			if ( y == 0 ||
					((y[2 * ll] <= qpData->options.equalityTolerance) && /* lower bound inactive */
					(y[2 * ll + 1] <= qpData->options.equalityTolerance))) /* upper bound inactive */
//			if ( y[2 * ll] * y[2 * ll + 1] > -qpData->options.activenessTolerance )
			{
				for (ii = 0; ii < dim0; ++ii) {
					for (jj = 0; jj < dim0; ++jj) {
						/* since M2 is dense, so is Z */
						res->data[ii * dim0 + jj] += Ztmp->data[ll * dim0 + ii]	* Ztmp->data[ll * dim0 + jj];
					}
				}
			} /* end of dyadic addend */
		}
	}
	else { /* diagonal H */
		/* Z already contains H^-1 * M2^T, therefore only multiplication with M2 from left is needed */
		/* compute M2 * Z as dyadic products */
		for (ll = 0; ll < dim1; ++ll) {
			/* only add columns of variables with inactive bounds */
			if (y == 0 ||
					((y[2 * ll] <= qpData->options.equalityTolerance) && /* lower bound inactive */
					(y[2 * ll + 1] <= qpData->options.equalityTolerance))) /* upper bound inactive */
			{
				for (ii = 0; ii < dim0; ++ii) {
					for (jj = 0; jj < dim0; ++jj) {
						/* since M2 is dense, so is Z */
						res->data[ii * dim0 + jj] += M2->data[ii * dim1 + ll] * Ztmp->data[ll * dim0 + jj];
					}
				}
			} /* end of dyadic addend */
		}
	}

	return QPDUNES_OK;
}
/*<<< END OF addMultiplyMatrixInvMatrixMatrixT */


/* ----------------------------------------------
 * Low level scalar product 
 * 
 >>>>>>                                           */
real_t scalarProd(	const vector_t* const x,
					const vector_t* const y,
					int_t len 	)
{
	int_t ii;
	real_t res = 0.;
	
	for( ii = 0; ii < len; ++ii ) {
		res += x->data[ii]*y->data[ii];
	}
	
	return res;
}
/*<<< END OF scalarProd */


/* ----------------------------------------------
 * ...
 * 
 >>>>>>                                           */
real_t vectorNorm(	const vector_t* const vec,
					int_t len
					)
{
	return sqrt( scalarProd( vec, vec, len ) );
}
/*<<< END OF vectorNorm */


/* ----------------------------------------------
 * Compute a Cholesky factorization of M
 * 
 >>>>>>                                           */
return_t factorizePosDefMatrix( 	qpData_t* const qpData,
									matrix_t* const cholM,
									const matrix_t* const M,
									int_t dim0
									)
{
	/** choose appropriate factorization routine */
	switch( M->sparsityType )	
	{
		case QPDUNES_DENSE		:
		case QPDUNES_SPARSE	:
			cholM->sparsityType = QPDUNES_DENSE;
			return denseCholeskyFactorization( qpData, cholM, M, dim0 );
		case QPDUNES_DIAGONAL	:
			/* cholM in this case is defined to contain full diagonal matrix (not a factor) */
			return qpDUNES_copyMatrix( cholM, M, dim0, dim0 );
		case QPDUNES_IDENTITY	:
			cholM->sparsityType = QPDUNES_IDENTITY;
			return QPDUNES_OK;
		default				:
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
}
/*<<< END OF factorizePosDefMatrix */


/* ----------------------------------------------
 * Compute a dense Cholesky factorization of M
 * 
#>>>>>                                            */
return_t denseCholeskyFactorization(	qpData_t* const qpData,
										matrix_t* const cholM,
										const matrix_t* const M,
										int_t dim0
										)
{
	int_t ii, jj, kk;
	real_t sum;
	
	/* go by columns */
	for( ii=0; ii<dim0; ++ii )
	{
		/* write diagonal element: jj == ii */
		sum = M->data[ii * dim0 + ii];
		
		for( kk = 0; kk < ii; ++kk ) {	/* subtract squared forepart of this row */
			sum -= cholM->data[ii * dim0 + kk] * cholM->data[ii * dim0 + kk];
		}
		
		if ( sum > qpData->options.QPDUNES_ZERO ) {
			cholM->data[ii * dim0 + ii] = sqrt( sum );
		}
		else {
			/* matrix not positive definite */
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Matrix not positive definite. Cholesky factorization could not be performed." );
			return QPDUNES_ERR_DIVISION_BY_ZERO;
		}
		
		/* write remainder of ii-th column */
		for( jj=(ii+1); jj<dim0; ++jj )
		{
			sum =  M->data[jj * dim0 + ii];
			
			for( kk = 0; kk < ii; ++kk ) {	/* subtract forepart of this row times forepart of ii-th row */
				sum -= cholM->data[ii * dim0 + kk] * cholM->data[jj * dim0 + kk];
			}
			
			cholM->data[jj * dim0 + ii] = sum / cholM->data[ii * dim0 + ii];
		}
	}
	
	return QPDUNES_OK;
}
/*<<< END OF denseCholeskyFactorization */



/*
 *	end of file
 */
