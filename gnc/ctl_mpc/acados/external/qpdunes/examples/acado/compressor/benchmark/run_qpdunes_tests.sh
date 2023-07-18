#!/bin/bash

echo ""
echo "Testing the qpDOOMED solver"
echo ""
echo ""

echo "Cleaning CMake shit and running CMake with Intel compiler"

mkdir build
mkdir build/simple_compressor_export
mkdir build/simple_compressor_export/qpoases

cd build
make clean
rm CMakeCache.txt
# export CC=icc CXX=icpc
export CC=clang CXX=clang
cmake -DQPSOLVER_TYPE=qpDUNES ..
# cmake -DQPSOLVER_TYPE=qpOASES ..

for N in $(seq 10 10 10); do
# for N in $(seq 10 10 100); do
	echo ""
	echo "Compiling tests for N = $N"
	echo ""

	# Generate the code
	../../build/simple_compressor $N
	
	# Build the code
# 	cd chain_mass_qpoases_2_export
# 	cmake -DQPSOLVER_TYPE=qpOASES .
# 	make clean
# 	make -j4 all
	
	# move the code
	cd build
	
	# Build the code
	make clean
	make -j4 all
	
	mv simple_compressor_test simple_compressor_test_qpdunes_N${N}
# 	mv simple_compressor_test simple_compressor_test_qpoases_N${N}
	
	# Run the code
# 	for k in $(seq 1 1 40); do
# 		let "ind = $N * 1000 + $k"
# 		./simple_compressor_test $ind
# 	done

done

echo ""
echo "Testing is over"
echo ""
