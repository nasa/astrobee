// CLASS_GEMM
//

void call_routines(struct RoutineArgs *args)
	{
	// routine call
	//
	BLASFEO(ROUTINE)(
		args->m, args->n, args->k,
		args->alpha,
		args->sA, args->ai, args->aj,
		args->sB, args->bi, args->bj,
		args->beta,
		args->sC, args->ci, args->cj,
		args->sD, args->di, args->dj);

	BLASFEO(REF(ROUTINE))(
		args->m, args->n, args->k,
		args->alpha,
		args->rA, args->ai, args->aj,
		args->rB, args->bi, args->bj,
		args->beta,
		args->rC, args->ci, args->cj,
		args->rD, args->di, args->dj);
	}



void print_routine(struct RoutineArgs *args)
	{
	printf("blasfeo_%s(%d, %d, %d, %f, A, %d, %d, B, %d, %d, %f, C, %d, %d, D, %d, %d);\n", string(ROUTINE), args->m, args->n, args->k, args->alpha, args->ai, args->aj, args->bi, args->bj, args->beta, args->ci, args->cj, args->di, args->dj);
	}



void print_routine_matrices(struct RoutineArgs *args)
	{
	if(!strcmp(string(ROUTINE), "dgemm_nn") || !strcmp(string(ROUTINE), "sgemm_nn"))
		{
		printf("\nPrint A:\n");
		blasfeo_print_xmat_debug(args->m, args->k, args->sA, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->m, args->k, args->rA, args->ai, args->aj, 0, 0, 0, "REF");

		printf("\nPrint B:\n");
		blasfeo_print_xmat_debug(args->k, args->n, args->sB, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->k, args->n, args->rB, args->ai, args->aj, 0, 0, 0, "REF");
		}
	if(!strcmp(string(ROUTINE), "dgemm_nt") || !strcmp(string(ROUTINE), "sgemm_nt"))
		{
		printf("\nPrint A:\n");
		blasfeo_print_xmat_debug(args->m, args->k, args->sA, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->m, args->k, args->rA, args->ai, args->aj, 0, 0, 0, "REF");

		printf("\nPrint B:\n");
		blasfeo_print_xmat_debug(args->n, args->k, args->sB, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->n, args->k, args->rB, args->ai, args->aj, 0, 0, 0, "REF");
		}
	if(!strcmp(string(ROUTINE), "dgemm_tn") || !strcmp(string(ROUTINE), "sgemm_tn"))
		{
		printf("\nPrint A:\n");
		blasfeo_print_xmat_debug(args->k, args->m, args->sA, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->k, args->m, args->rA, args->ai, args->aj, 0, 0, 0, "REF");

		printf("\nPrint B:\n");
		blasfeo_print_xmat_debug(args->k, args->n, args->sB, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->k, args->n, args->rB, args->ai, args->aj, 0, 0, 0, "REF");
		}
	if(!strcmp(string(ROUTINE), "dgemm_tt") || !strcmp(string(ROUTINE), "sgemm_tt"))
		{
		printf("\nPrint A:\n");
		blasfeo_print_xmat_debug(args->k, args->m, args->sA, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->k, args->m, args->rA, args->ai, args->aj, 0, 0, 0, "REF");

		printf("\nPrint B:\n");
		blasfeo_print_xmat_debug(args->n, args->k, args->sB, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->n, args->k, args->rB, args->ai, args->aj, 0, 0, 0, "REF");
		}

	printf("\nPrint C:\n");
	blasfeo_print_xmat_debug(args->m, args->n, args->sC, args->ai, args->aj, 0, 0, 0, "HP");
	blasfeo_print_xmat_debug(args->m, args->n, args->rC, args->ai, args->aj, 0, 0, 0, "REF");

	printf("\nPrint D:\n");
	blasfeo_print_xmat_debug(args->m, args->n, args->sD, args->ai, args->aj, 0, 0, 0, "HP");
	blasfeo_print_xmat_debug(args->m, args->n, args->rD, args->ai, args->aj, 0, 0, 0, "REF");
	}



void set_test_args(struct TestArgs *targs)
	{
#if defined(MF_PANELMAJ)
	targs->ais = 5;
	targs->bis = 5;
	targs->dis = 5;
	targs->xjs = 2;
#endif

#if defined(TARGET_X64_INTEL_SKYLAKE_X)
	targs->nis = 25;
	targs->njs = 25;
#else
	targs->nis = 13;
	targs->njs = 5;
#endif
	targs->nks = 6;

	targs->alphas = 1;
	}
