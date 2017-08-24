px4_add_module(
	MODULE morus_modules__morus_ss_control
	MAIN morus_ss_control
	STACK_MAIN 1200 # IMPORTANT!!!, defines stack size for this part
	COMPILE_FLAGS
	SRCS
		SSControl.cpp
		morus_ss_control_main.cpp
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
