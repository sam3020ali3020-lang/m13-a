############################################################################
#
# XQPOWER CAN Servo Driver
#
############################################################################

px4_add_module(
	MODULE drivers__xqpower_can
	MAIN xqpower_can
	SRCS
		XqpowerCan.cpp
		XqpowerCan.hpp
	DEPENDS
	)
