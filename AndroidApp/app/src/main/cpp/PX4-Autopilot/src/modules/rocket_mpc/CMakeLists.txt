############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# ╔══════════════════════════════════════════════════════════════════╗
# ║  DUAL-BUILD WARNING                                            ║
# ║                                                                ║
# ║  This CMakeLists.txt is used ONLY by the PX4 SITL/Linux build  ║
# ║  (make px4_sitl_default).  It is NOT used by the Android NDK   ║
# ║  build, which has its own parallel source list at:              ║
# ║                                                                ║
# ║    AndroidApp/app/src/main/cpp/CMakeLists.txt                  ║
# ║    → PX4_SOURCES  (lines ~873-880)   module .cpp files         ║
# ║    → m130_solvers  IMPORTED static lib (pre-built for arm64)   ║
# ║                                                                ║
# ║  If you ADD or REMOVE a source file here, you MUST also update ║
# ║  the Android CMakeLists.txt, or the Android build will be      ║
# ║  missing / carrying stale object files.                        ║
# ╚══════════════════════════════════════════════════════════════════╝
#
# rocket_mpc depends on acados + hpipm + blasfeo shared libraries
# (libacados.so, libhpipm.so, libblasfeo.so) and on Linux/Android-only
# scheduling APIs (sched_setaffinity, SYS_gettid, /sys/devices/.../cpu_capacity).
# Dynamic linking and those APIs do not exist on NuttX targets (Pixhawk / STM32),
# so this module is POSIX-only by design. Skip the entire build on NuttX to
# prevent accidental enablement via a board Kconfig from breaking firmware builds.
if("${PX4_PLATFORM}" STREQUAL "nuttx")
	message(STATUS "rocket_mpc: skipped on NuttX (POSIX/Linux-only module)")
	return()
endif()

# Paths to acados and generated solver code.
#
# Resolution order (first match wins):
#   1. CMake cache variable (e.g. `-DACADOS_ROOT=/path`)
#   2. Environment variable (`ACADOS_ROOT`, `M130_SOLVER_ROOT`)
#   3. Repo-relative default: `<m13>/acados-main` and `<m13>/c_generated_code`
#
# The repo root is 6 levels above PX4_SOURCE_DIR:
#   PX4-Autopilot / cpp / main / src / app / AndroidApp / → m13
set(M13_REPO_ROOT "${PX4_SOURCE_DIR}/../../../../../..")

if(NOT DEFINED ACADOS_ROOT)
	if(DEFINED ENV{ACADOS_ROOT})
		set(ACADOS_ROOT $ENV{ACADOS_ROOT})
	else()
		set(ACADOS_ROOT "${M13_REPO_ROOT}/acados-main")
	endif()
endif()

if(NOT DEFINED M130_SOLVER_ROOT)
	if(DEFINED ENV{M130_SOLVER_ROOT})
		set(SOLVER_ROOT $ENV{M130_SOLVER_ROOT})
	else()
		set(SOLVER_ROOT "${M13_REPO_ROOT}/c_generated_code")
	endif()
else()
	set(SOLVER_ROOT "${M130_SOLVER_ROOT}")
endif()

# Normalise to absolute paths so downstream target_include_directories /
# target_link_libraries calls do not depend on the current source dir.
get_filename_component(ACADOS_ROOT "${ACADOS_ROOT}" ABSOLUTE)
get_filename_component(SOLVER_ROOT "${SOLVER_ROOT}" ABSOLUTE)

# Collect generated solver C sources (MPC only — MHE removed)
set(SOLVER_SOURCES
	${SOLVER_ROOT}/acados_solver_m130_rocket.c
	${SOLVER_ROOT}/acados_sim_solver_m130_rocket.c
	${SOLVER_ROOT}/m130_rocket_model/m130_rocket_expl_ode_fun.c
	${SOLVER_ROOT}/m130_rocket_model/m130_rocket_expl_vde_adj.c
	${SOLVER_ROOT}/m130_rocket_model/m130_rocket_expl_vde_forw.c
)

# Build solver sources as a static library (they are plain C, not PX4 module code)
add_library(m130_solvers STATIC ${SOLVER_SOURCES})
target_include_directories(m130_solvers PUBLIC
	${SOLVER_ROOT}
	${SOLVER_ROOT}/m130_rocket_model
	${ACADOS_ROOT}/include
	${ACADOS_ROOT}/include/acados
	${ACADOS_ROOT}/include/blasfeo/include
	${ACADOS_ROOT}/include/hpipm/include
)
target_link_libraries(m130_solvers PUBLIC
	${ACADOS_ROOT}/lib/libacados.so
	${ACADOS_ROOT}/lib/libhpipm.so
	${ACADOS_ROOT}/lib/libblasfeo.so
	m
)
# Suppress warnings in generated code
target_compile_options(m130_solvers PRIVATE -w)

px4_add_module(
	MODULE modules__rocket_mpc
	MAIN rocket_mpc
	STACK_MAX 8192
	SRCS
		RocketMPC.cpp
		mpc_controller.cpp
		los_guidance.cpp
		sensor_bridge.cpp
	INCLUDES
		${SOLVER_ROOT}
		${ACADOS_ROOT}/include
		${ACADOS_ROOT}/include/acados
		${ACADOS_ROOT}/include/blasfeo/include
		${ACADOS_ROOT}/include/hpipm/include
	DEPENDS
		geo
		mathlib
		px4_work_queue
	)

target_link_libraries(modules__rocket_mpc PRIVATE m130_solvers)
