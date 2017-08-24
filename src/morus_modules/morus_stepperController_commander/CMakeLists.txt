#morus_stepperController_commander module 
px4_add_module(
	MODULE morus_modules__morus_stepperController_commander
	MAIN morus_stepperController_commander
	STACK_MAIN 1200 # IMPORTANT!!!, defines stack size for this part
	COMPILE_FLAGS
	SRCS
		morus_stepperController_commander_main.cpp
		StepperControllerCommander.cpp
		StepperControllerParams.cpp
	INCLUDES
		PUBLIC ${PX4_SOURCE_DIR}/src/morus_lib/serial_lib/
	DEPENDS
		platforms__common
	)
#Linking the serial library with example code module	
target_link_libraries(morus_modules__morus_stepperController_commander morus_lib__serial_lib)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
