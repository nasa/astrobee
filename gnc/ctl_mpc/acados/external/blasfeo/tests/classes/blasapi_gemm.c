// CLASS_GEMM
//

void call_routines(struct RoutineArgs *args)
	{
	BLASFEO_BLAS(ROUTINE)(
		string(TRANSA), string(TRANSB),
		&(args->m), &(args->n), &(args->k), &(args->alpha),
		args->cA->pA, &(args->cA->m),
		args->cB->pA, &(args->cB->m), &(args->beta),
		args->cD->pA, &(args->cD->m));

	BLAS(ROUTINE)(
		string(TRANSA), string(TRANSB),
		&(args->m), &(args->n), &(args->k), &(args->alpha),
		args->rA->pA, &(args->rA->m),
		args->rB->pA, &(args->rB->m), &(args->beta),
		args->rD->pA, &(args->rD->m));

	}



void print_routine(struct RoutineArgs *args)
	{
	printf("blas_%s(%s, %s, %d, %d, %d, %f, A, %d, B, %d, C, %d);\n", string(ROUTINE), string(TRANSA), string(TRANSB), args->m, args->n, args->k, args->alpha, args->cA->m, args->cB->m, args->cD->m);
	}



void print_routine_matrices(struct RoutineArgs *args)
	{
	// TODO fixed based on transA & transB
	printf("\nPrint A:\n");
	print_xmat_debug(args->m, args->n, args->cA, args->ai, args->aj, 0, 0, 0);
	print_xmat_debug(args->m, args->n, args->rA, args->ai, args->aj, 0, 0, 0);

	printf("\nPrint B:\n");
	print_xmat_debug(args->m, args->n, args->cB, args->ai, args->aj, 0, 0, 0);
	print_xmat_debug(args->m, args->n, args->rB, args->ai, args->aj, 0, 0, 0);

	printf("\nPrint C:\n");
	print_xmat_debug(args->m, args->n, args->cC, args->ai, args->aj, 0, 0, 0);
	print_xmat_debug(args->m, args->n, args->rC, args->ai, args->aj, 0, 0, 0);

	printf("\nPrint D:\n");
	print_xmat_debug(args->m, args->n, args->cD, args->ai, args->aj, 0, 0, 0);
	print_xmat_debug(args->m, args->n, args->rD, args->ai, args->aj, 0, 0, 0);
	}



void set_test_args(struct TestArgs *targs)
	{
//	targs->ais = 5;
//	targs->bis = 5;
//	targs->dis = 5;
//	targs->xjs = 2;

	targs->nis = 17;
	targs->njs = 8;
	targs->nks = 8;

	targs->alphas = 1;
	}
