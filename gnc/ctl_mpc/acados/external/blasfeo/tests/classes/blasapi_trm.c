// CLASS_GEMM

void call_routines(struct RoutineArgs *args)
	{

//	#if (VERBOSE>2)
//	printf("Calling BLASFEO implementation\n");
//	#endif

	BLASFEO_BLAS(ROUTINE)(
		string(SIDE), string(UPLO), string(TRANSA), string(DIAG),
		&(args->m), &(args->n), &(args->alpha),
		args->cA->pA, &(args->cA->m),
		args->cB->pA, &(args->cB->m));

//	#if (VERBOSE>2)
//	printf("Calling test reference implementation\n");
//	#endif

	BLAS(ROUTINE)(
		string(SIDE), string(UPLO), string(TRANSA), string(DIAG),
		&(args->m), &(args->n), &(args->alpha),
		args->rA->pA, &(args->rA->m),
		args->rB->pA, &(args->rB->m));

	}


void print_routine(struct RoutineArgs *args)
	{
	printf("blas_%s(%s, %s, %s, %s, %d, %d, %f, A, %d, C, %d);\n", string(ROUTINE), string(UPLO), string(SIDE), string(TRANSA), string(DIAG), args->m, args->n, args->alpha, args->cA->m, args->cB->m);
	}



void print_routine_matrices(struct RoutineArgs *args)
	{
	// TODO fixed based on side
	printf("\nPrint A:\n");
	print_xmat_debug(args->m, args->n, args->cA, 0, 0, 0, 0, 0);
	print_xmat_debug(args->m, args->n, args->rA, 0, 0, 0, 0, 0);

	printf("\nPrint B:\n");
	print_xmat_debug(args->m, args->n, args->cB, 0, 0, 0, 0, 0);
	print_xmat_debug(args->m, args->n, args->rB, 0, 0, 0, 0, 0);
	}



void set_test_args(struct TestArgs *targs)
	{
	targs->nis = 21;
	targs->njs = 21;
//	targs->nks = 20;

//	targs->ni0 = 10;
//	targs->nj0 = 10;
//	targs->nk0 = 10;

//	targs->alphas = 1;
	}
