px4_add_module(
    MODULE morus_modules__morus_direct_control
    MAIN morus_direct_control
    STACK_MAIN 3000 # IMPORTANT!!!, defines stack size for this part
    STACK_MAX 1600
    COMPILE_FLAGS
    SRCS
        DirectControl.cpp
        morus_direct_control_main.cpp
    DEPENDS
        platforms__common
    )
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
