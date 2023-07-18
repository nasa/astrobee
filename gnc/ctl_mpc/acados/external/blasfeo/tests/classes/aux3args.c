// CLASS_3ARGS
//
// blasfeo_xgecpsc(ni, mi, alpha, &sA, ai, aj, &sB, bi, bj);
// blasfeo_xgead(ni, mi, alpha, &sA, ai, aj, &sB, bi, bj);

void call_routines(struct RoutineArgs *args){
	// call HP and REF routine

	// routine call
	//
	BLASFEO(ROUTINE)(
		args->n, args->m, args->alpha,
		args->sA, args->ai, args->aj,
		args->sB, args->bi, args->bj
		);

	BLASFEO(REF(ROUTINE))(
		args->n, args->m, args->alpha,
		args->rA, args->ai, args->aj,
		args->rB, args->bi, args->bj
		);
}

void print_routine(struct RoutineArgs *args){
	// print current class signature

	printf("%s ", string(ROUTINE));
	printf(
		"B[%d:%d,%d:%d] =  %f*A[%d:%d,%d:%d]\n",
		args->bi, args->m, args->bj, args->n,
		args->alpha, args->ai, args->m, args->aj, args->n
	);

}

void print_routine_matrices(struct RoutineArgs *args){

		printf("\nPrint A:\n");
		blasfeo_print_xmat_debug(args->m, args->n, args->sA, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->m, args->n, args->rA, args->ai, args->aj, 0, 0, 0, "REF");

		printf("\nPrint B:\n");
		blasfeo_print_xmat_debug(args->m, args->n, args->sB, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->m, args->n, args->rB, args->ai, args->aj, 0, 0, 0, "REF");
}

void set_test_args(struct TestArgs *targs)
{
	targs->ais = 5;
	targs->bis = 5;
	targs->dis = 5;
	targs->xjs = 2;

	targs->nis = 5;
	targs->njs = 5;
	targs->nks = 5;

	targs->alphas = 1;
}
