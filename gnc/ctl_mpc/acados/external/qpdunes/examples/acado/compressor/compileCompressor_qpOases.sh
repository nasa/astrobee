#!/bin/bash

echo ""
echo "Compiling for the qpOASES solver"
echo ""
echo ""

echo "Cleaning CMake shit and running CMake with Intel compiler"
cd build
make clean
rm CMakeCache.txt
# export CC=icc CXX=icpc
export CC=clang CXX=clang
# rm simple_compressor_export/build/simple_compressor_test_qpoases_N
cmake -DQPSOLVER_TYPE=qpOASES ..
make -j4 all


for N in $(seq 10 10 100); do
	echo ""
	echo "Compiling tests for N = $N"
		echo ""

	# Generate the code
	./simple_compressor $N
	
	# Build the code
	cd simple_compressor_export
	cmake -DQPSOLVER_TYPE=qpOASES .
	make clean
	make -j4 all
	
	# move the code
	mv simple_compressor_test build/simple_compressor_test_qpoases_N${N}
	cd ..
	
done

echo ""
echo "Compiling is over"
echo ""
