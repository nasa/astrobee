// CLASS_GETRF_BLASAPI
//
void call_routines(struct RoutineArgs *args)
	{

	// copy input matrix A in C
	GECP_REF(args->m, args->m, args->cA, 0, 0, args->cC, 0, 0);
	GECP_REF(args->m, args->m, args->rA, 0, 0, args->rC, 0, 0);

	// routine call
	//
	BLASFEO_BLAS(ROUTINE)(
		&(args->m), &(args->n),
		args->cC->pA, &(args->cC->m),
		args->cipiv, &(args->info));

	BLAS(ROUTINE)(
		&(args->m), &(args->n),
		args->rC->pA, &(args->rC->m),
		args->ripiv, &(args->info));

	// C matrix is overwritten with the solution

	// copy result matrix C in D
	GECP_REF(args->m, args->m, args->cC, 0, 0, args->cD, 0, 0);
	GECP_REF(args->m, args->m, args->rC, 0, 0, args->rD, 0, 0);

	}



void print_routine(struct RoutineArgs *args)
	{
	printf("blas_%s(%d, %d, A, %d, ipiv, info);\n", string(ROUTINE), args->m, args->n, args->cC->m);
	}



void print_routine_matrices(struct RoutineArgs *args)
	{
	printf("\nInput matrix:\n");
	print_xmat_debug(args->m, args->n, args->cA, 0, 0, 0, 0, 0);
	print_xmat_debug(args->m, args->n, args->rA, 0, 0, 0, 0, 0);

	printf("\nRow pivot vector:\n");
	int size = args->m < args->n ? args->m : args->n;
	int_print_mat(1, size, args->cipiv, 1);
	int_print_mat(1, size, args->ripiv, 1);
	}



void set_test_args(struct TestArgs *targs)
	{
	targs->nis = 21;
	targs->njs = 21;
	}
