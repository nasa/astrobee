// CLASS_1ARGS
//
// blasfeo_xgesc(ni, mi, &sA, ai, aj);

void call_routines(struct RoutineArgs *args){

	// unpack args

	// routine call
	//
	BLASFEO(ROUTINE)(
		args->n, args->m, args->alpha, args->sA, args->ai, args->aj
		);

	BLASFEO(REF(ROUTINE))(
		args->n, args->m, args->alpha, args->rA, args->ai, args->aj
		);

}

void print_routine(struct RoutineArgs *args){
	// unpack args

	printf("%s\n", string(ROUTINE));
	printf(
		"A[%d:%d,%d:%d] =  %f*A[%d:%d,%d:%d]\n",
		args->ai, args->m, args->aj, args->n,
		args->alpha, args->ai, args->m, args->aj, args->n
	);

}

void print_routine_matrices(struct RoutineArgs *args){

		printf("\nPrint A:\n");
		blasfeo_print_xmat_debug(args->m, args->n, args->sA, args->ai, args->aj, 0, 0, 0, "HP");
		blasfeo_print_xmat_debug(args->m, args->n, args->rA, args->ai, args->aj, 0, 0, 0, "REF");
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
