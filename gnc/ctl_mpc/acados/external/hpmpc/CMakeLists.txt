cmake_minimum_required(VERSION 2.8.11)
project(hpmpc)

# target architecture. currently supported (more architectures are available with the older v0.1 release):
# X64_AVX2   : machine with AVX2 and FMA3 instruction sets (recent Intel and AMD processors), 64 bit operating system, code optimized for Intel Haswell.
# X64_AVX    : machine with AVX instruction set (previous generation Intel and AMD processors), 64 bit operating system, code optimized for Intel Sandy-Bridge.
# X64_SSE3   : machine with SSE3 instruction set (older Intel and AMD processors), 64 bit operating system, code optimized for Intel Core.
# CORTEX_A57 : machine with ARMv8a processor with NEON, 64-bit operating system, code optimized for ARM Cortex A57.
# CORTEX_A15 : machine with ARMv7a processor with VPFv3 (D32 versions) and NEON, code optimized for ARM Cortex A15.
# CORTEX_A9  : machine with ARMv7a processor with VPFv3 (D32 versions) and NEON, code optimized for ARM Cortex A9.
# CORTEX_A7  : machine with ARMv7a processor with VPFv3 (D32 versions) and NEON, code optimized for ARM Cortex A7.
# C99_4X4    : c99 reference code, performing better on a machine with at least 32 scalar registers.

# set(TARGET X64_AVX)
set(TARGET C99_4X4)

# use BLASFEO?
set(USE_BLASFEO 1)
set(BLASFEO_PATH ${PROJECT_SOURCE_DIR}/../blasfeo)

# headers installation directory
set(HPMPC_HEADERS_INSTALLATION_DIRECTORY "include" CACHE STRING "Headers local installation directory")

# C Compiler
# set(CC_COMPILER gcc)
# set(MINGW_ROOT ~/cross_compiler_windows)  (can be obtained here: https://sourceforge.net/projects/mingw-w64/files/)
# set(CC_COMPILER ${MINGW_ROOT}/mingw-w64-bin_x86_64-linux_20131228/bin/x86_64-w64-mingw32-gcc)
# set(CC_COMPILER /opt/gcc/bin/gcc) # custom gcc
# set(CC_COMPILER clang)

# operative system
set(OS LINUX)
# set(OS WINDOWS)

# installation directory
set(PREFIX /usr)

# clear compiler flags
set(CMAKE_C_FLAGS "")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -fPIC")

if(${USE_BLASFEO} MATCHES 1)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -fPIC -DBLASFEO -I${BLASFEO_PATH}/include")
endif(${USE_BLASFEO} MATCHES 1)

if(${TARGET} MATCHES C99_4X4)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DTARGET_C99_4X4")
endif(${TARGET} MATCHES C99_4X4)

# common optimization/debugging flags
set(COMMON_FLAGS "-g -fPIC")

if(${OS} MATCHES WINDOWS)
	set(COMMON_FLAGS "${COMMON_FLAGS} -DOS_WINDOWS")
endif(${OS} MATCHES WINDOWS)

set(DEBUG "-pedantic -Wfloat-equal")
# set(LDFLAGS "")

# reference code linking to standard BLAS and LAPACK
# 0
# OPENBLAS
# BLIS
# NETLIB
# MKL

# reference implementation
set(REF_BLAS 0)
if(${REF_BLAS} MATCHES 0)
	set(REF_BLAS_FLAGS "")
endif(${REF_BLAS} MATCHES 0)

if(${REF_BLAS} MATCHES OPENBLAS)
	set(REF_BLAS_FLAGS -DREF_BLAS_OPENBLAS)
endif(${REF_BLAS} MATCHES OPENBLAS)

if(${REF_BLAS} MATCHES BLIS)
	set(REF_BLAS_FLAGS -DREF_BLAS_BLIS)
endif(${REF_BLAS} MATCHES BLIS)

if(${REF_BLAS} MATCHES NETLIB)
	set(REF_BLAS_FLAGS -DREF_BLAS_NETLIB)
endif(${REF_BLAS} MATCHES NETLIB)

if(${REF_BLAS} MATCHES MKL)
	set(REF_BLAS_FLAGS -DREF_BLAS_MKL)
	set(COMMON_FLAGS "${COMMON_FLAGS} -m64 -I/opt/intel/mkl/include")
endif(${REF_BLAS} MATCHES MKL)

# architecture-specific optimization flags
if(${TARGET} MATCHES X64_AVX2)
	set(CFLAGS "$(COMMON_FLAGS) -m64 -mavx2 -mfma -DTARGET_X64_AVX2 $(REF_BLAS_FLAGS) $(DEBUG}")
endif(${TARGET} MATCHES X64_AVX2)

if(${TARGET} MATCHES X64_AVX)
	set(CFLAGS "${COMMON_FLAGS} -m64 -mavx -DTARGET_X64_AVX ${REF_BLAS_FLAGS} ${DEBUG}")
endif(${TARGET} MATCHES X64_AVX)

if(${TARGET} MATCHES X64_SSE3)
	set(CFLAGS "${COMMON_FLAGS} -m64 -msse3 -DTARGET_X64_SSE3 ${REF_BLAS_FLAGS} ${DEBUG}")
endif(${TARGET} MATCHES X64_SSE3)

if(${TARGET} MATCHES C99_4X4)
	set(CFLAGS "${COMMON_FLAGS} -DTARGET_C99_4X4 ${REF_BLAS_FLAGS} ${DEBUG}")
endif(${TARGET} MATCHES C99_4X4)

if(${TARGET} MATCHES C99_4X4_PREFETCH)
	set(CFLAGS "${COMMON_FLAGS} -march=native -DTARGET_C99_4X4_PREFETCH ${REF_BLAS_FLAGS} $(DEBUG)")
endif(${TARGET} MATCHES C99_4X4_PREFETCH)

if(${TARGET} MATCHES CORTEX_A57)
	set(CFLAGS "${COMMON_FLAGS} -march=armv8-a+fp+simd -mcpu=cortex-a57 -DTARGET_CORTEX_A57 ${REF_BLAS_FLAGS} ${DEBUG}")
endif(${TARGET} MATCHES CORTEX_A57)

if(${TARGET} MATCHES CORTEX_A15)
	set(CFLAGS "${COMMON_FLAGS} -marm -mfloat-abi=hard -mfpu=neon-vfpv4 -mcpu=cortex-a15 -DTARGET_CORTEX_A15 ${REF_BLAS_FLAGS} $(DEBUG)")
endif(${TARGET} MATCHES CORTEX_A15)

if(${TARGET} MATCHES CORTEX_A9)
	set(CFLAGS "${COMMON_FLAGS} -marm -mfloat-abi=hard -mfpu=neon -mcpu=cortex-a9 -DTARGET_CORTEX_A9 ${REF_BLAS_FLAGS} ${DEBUG}")
endif(${TARGET} MATCHES CORTEX_A9)

if(${TARGET} MATCHES CORTEX_A7)
	set(CFLAGS "${COMMON_FLAGS} -marm -mfloat-abi=hard -mfpu=neon-vfpv4 -mcpu=cortex-a7 -DTARGET_CORTEX_A7 ${REF_BLAS_FLAGS} ${DEBUG}")
endif(${TARGET} MATCHES CORTEX_A7)

# source files
if(${TARGET} MATCHES C99_4X4)
	file(GLOB HPMPC_AUXILIARY_SRC
		${PROJECT_SOURCE_DIR}/auxiliary/d_aux_lib4.c
		${PROJECT_SOURCE_DIR}/auxiliary/d_aux_extern_depend_lib4.c
		${PROJECT_SOURCE_DIR}/auxiliary/i_aux.c)
else(${TARGET} MATCHES C99_4X4)
	file(GLOB HPMPC_AUXILIARY_SRC
	"")
endif(${TARGET} MATCHES C99_4X4)

if(${USE_BLASFEO} MATCHES 1)
	file(GLOB HPMPC_LQCP_SOLVERS_SRC
		${PROJECT_SOURCE_DIR}/lqcp_solvers/d_back_ric_rec_libstr.c
		${PROJECT_SOURCE_DIR}/lqcp_solvers/d_part_cond_libstr.c)

	file(GLOB HPMPC_MPC_AUXILIARY_SRC
		${PROJECT_SOURCE_DIR}/mpc_solvers/c99/d_aux_ip_hard_libstr.c)

	file(GLOB HPMPC_MPC_SOLVERS_SRC
		${PROJECT_SOURCE_DIR}/mpc_solvers/d_ip2_res_hard_libstr.c)

	file(GLOB HPMPC_MPC_INTERFACES_SRC
		${PROJECT_SOURCE_DIR}/interfaces/c/fortran_order_interface_libstr.c
		${PROJECT_SOURCE_DIR}/mpc_solvers/d_tree_ip2_res_hard_libstr.c
		${PROJECT_SOURCE_DIR}/mpc_solvers/d_res_ip_res_hard_libstr.c
		${PROJECT_SOURCE_DIR}/mpc_solvers/d_tree_res_ip_res_hard_libstr.c)

	set(HPMPC_SRC
		${HPMPC_AUXILIARY_SRC}
		${HPMPC_LQCP_SOLVERS_SRC}
		${HPMPC_MPC_AUXILIARY_SRC}
		${HPMPC_MPC_SOLVERS_SRC}
		${HPMPC_MPC_SOLVERS_C99_SRC}
		${HPMPC_MPC_INTERFACES_SRC})

else(${USE_BLASFEO} MATCHES 1)
	file(GLOB HPMPC_KERNEL_SRC
		${PROJECT_SOURCE_DIR}/kernel/c99/kernel_dgemm_c99_lib4.c
		${PROJECT_SOURCE_DIR}/kernel/c99/kernel_dtrmm_c99_lib4.c
		${PROJECT_SOURCE_DIR}/kernel/c99/kernel_dtrsm_c99_lib4.c
		${PROJECT_SOURCE_DIR}/kernel/c99/kernel_dsyrk_c99_lib4.c
		${PROJECT_SOURCE_DIR}/kernel/c99/kernel_dpotrf_c99_lib4.c
		${PROJECT_SOURCE_DIR}/kernel/c99/kernel_dgemv_c99_lib4.c
		${PROJECT_SOURCE_DIR}/kernel/c99/kernel_dtrmv_c99_lib4.c
		${PROJECT_SOURCE_DIR}/kernel/c99/kernel_dtrsv_c99_lib4.c
		${PROJECT_SOURCE_DIR}/kernel/c99/kernel_dsymv_c99_lib4.c
		${PROJECT_SOURCE_DIR}/kernel/c99/kernel_dtran_c99_lib4.c
		${PROJECT_SOURCE_DIR}/kernel/c99/kernel_dttmm_c99_lib4.c
		${PROJECT_SOURCE_DIR}/kernel/c99/kernel_dtrinv_c99_lib4.c
		${PROJECT_SOURCE_DIR}/kernel/c99/kernel_dcopy_c99_lib4.c
		${PROJECT_SOURCE_DIR}/kernel/c99/kernel_dgetrf_c99_lib4.c)

	file(GLOB HPMPC_BLAS_SRC
		${PROJECT_SOURCE_DIR}/blas/blas_d_lib4.c)

	file(GLOB HPMPC_LQCP_SOLVERS_SRC
		${PROJECT_SOURCE_DIR}/lqcp_solvers/d_back_ric_rec.c
		${PROJECT_SOURCE_DIR}/lqcp_solvers/d_for_schur_rec.c
		${PROJECT_SOURCE_DIR}/lqcp_solvers/d_res.c
		${PROJECT_SOURCE_DIR}/lqcp_solvers/d_part_cond.c)

	file(GLOB HPMPC_MPC_AUXILIARY_SRC
		${PROJECT_SOURCE_DIR}/mpc_solvers/c99/d_aux_ip_hard_lib4.c
		${PROJECT_SOURCE_DIR}/mpc_solvers/c99/d_res_ip_res_hard.c
		${PROJECT_SOURCE_DIR}/mpc_solvers/c99/d_aux_ip_soft_lib4.c)

	file(GLOB HPMPC_MPC_SOLVERS_SRC
		${PROJECT_SOURCE_DIR}/mpc_solvers/d_ip2_hard.c
		${PROJECT_SOURCE_DIR}/mpc_solvers/d_res_ip_hard.c
		${PROJECT_SOURCE_DIR}/mpc_solvers/d_ip2_res_hard.c
		${PROJECT_SOURCE_DIR}/mpc_solvers/d_ip2_soft.c
		${PROJECT_SOURCE_DIR}/mpc_solvers/d_res_ip_soft.c)

	file(GLOB HPMPC_MPC_INTERFACES_SRC
		${PROJECT_SOURCE_DIR}/interfaces/c/c_interface_work_space.c
		${PROJECT_SOURCE_DIR}/interfaces/c/c_order_interface.c
		${PROJECT_SOURCE_DIR}/interfaces/c/fortran_order_interface.c)

	set(HPMPC_SRC
		${HPMPC_AUXILIARY_SRC}
		${HPMPC_KERNEL_SRC}
		${HPMPC_BLAS_SRC}
		${HPMPC_LCQP_SOLVERS_SRC}
		${HPMPC_MPC_AUXILIARY_SRC}
		${HPMPC_MPC_SOLVERS_SRC}
		${HPMPC_MPC_SOLVERS_C99_SRC}
		${HPMPC_MPC_INTERFACES_SRC})

endif(${USE_BLASFEO} MATCHES 1)

# configure
if(${TARGET} MATCHES X64_AVX2)
	set(TARGET_DEF TARGET_X64_AVX2)
endif(${TARGET} MATCHES X64_AVX2)

if(${TARGET} MATCHES X64_AVX)
	set(TARGET_DEF TARGET_X64_AVX)
endif(${TARGET} MATCHES X64_AVX)

if(${TARGET} MATCHES X64_SSE3)
	set(TARGET_DEF TARGET_X64_SSE3)
endif(${TARGET} MATCHES X64_SSE3)

if(${TARGET} MATCHES C99_4X4)
	set(TARGET_DEF TARGET_C99_4X4)
endif(${TARGET} MATCHES C99_4X4)

if(${TARGET} MATCHES CORTEX_A57)
	set(TARGET_DEF TARGET_CORTEX_A57)
endif(${TARGET} MATCHES CORTEX_A57)

if(${TARGET} MATCHES CORTEX_A15)
	set(TARGET_DEF TARGET_CORTEX_A15)
endif(${TARGET} MATCHES CORTEX_A15)

if(${TARGET} MATCHES CORTEX_A9)
	set(TARGET_DEF TARGET_CORTEX_A9)
endif(${TARGET} MATCHES CORTEX_A9)

if(${TARGET} MATCHES CORTEX_A7)
	set(TARGET_DEF TARGET_CORTEX_A7)
endif(${TARGET} MATCHES CORTEX_A7)

if(${USE_BLASFEO} MATCHES 1)
	set(BLASFEO_DEF BLASFEO)
else(${USE_BLASFEO} MATCHES 1)
	set(BLASFEO_DEF WITHOUT_BLASFEO)
endif(${USE_BLASFEO} MATCHES 1)

configure_file(${PROJECT_SOURCE_DIR}/include/target.h.in
	${CMAKE_CURRENT_SOURCE_DIR}/include/target.h @ONLY)

# add library
add_library(hpmpc STATIC ${HPMPC_SRC})
target_link_libraries(hpmpc blasfeo)

install(TARGETS hpmpc EXPORT hpmpcConfig
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin)

install(EXPORT hpmpcConfig DESTINATION cmake)

file(GLOB_RECURSE HPMPC_HEADERS "include/*.h")
install(FILES ${HPMPC_HEADERS} DESTINATION ${HPMPC_HEADERS_INSTALLATION_DIRECTORY})
