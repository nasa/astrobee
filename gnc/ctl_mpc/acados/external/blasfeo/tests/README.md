## Test Framework

In this test framework every routine is called with different
combinations of arguments,
the result is stored and compared with the result of the reference
implementation of the same routine called with the safe arguments.

The execution pipeline in this framework aims both to
achieve both fast execution and comparison of similar calls and both to
allow great flexibility and scalability of test specifications.

A gradient of abstraction from the slower and more
flexible code (Python, json) to the fast and efficient (C and assembly).

### Test definition

- `test_schema.json`:
	Define all possible tests

- `batch_run.json`:
	Define current test run to be executed
	Check validity against the schema

### Build and execution

- `tester.py` :
	Python script to build the test recipe from the definition and call the right make
	command

- `Makefile`:
	Generates compiler command and call the compiler

### Test C code implementation

- `test_{s,d}_{aux,blas1,blas2,blas3}.c`:
	Compilation target interface with Makefile

- `test_{s,d}_common.h`:
	Define precision related macros

- `test_x_common.{h,c}`:
	Include test C helper functions

- `test_class_{gemm, ..}.c`:
	Define variations of test helper function like `call_routine`
	tailored to the specific routine class i.e. `gemm`,
	every class of routines have the same signature.

- `test_x.c` :
	Run the actual test templated with the aforementioned configurations.


### BLASFEO

- `blasfeo_libref.{a, so}`:

	Blasfeo library compiled with REFERENCE target with routines name aliased
	postponing `_ref` prefix in order to coesist with the same routines
	compiled with other targets, i.e. HIGH_PERFORMANCE.

- `blasfeo_lib.{a, so}`:

	The actual code to be tested


# How To

Run: `python tester.py`

The default behavior is to run `recipe_default.json`
which test only `dgemm_nn` routine for target `GENERIC`.

To customize the run, follow the next two steps:
- Edit the configuration file `recipe_all.json` with the desired values
- Run: `python tester.py <your_recipe.json>`

NB: Only the routines specified in `recipe_all.json` are supported at
now.
