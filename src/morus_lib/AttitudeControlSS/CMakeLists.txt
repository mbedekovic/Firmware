px4_add_module(
	MODULE morus_lib__AttitudeControlSS
	COMPILE_FLAGS
	SRCS
		AttitudeControlSS.cpp
		AttitudeFeedForward.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
