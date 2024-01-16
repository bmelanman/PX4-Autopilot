############################################################################
#
#   Copyright (c) 2024 Gimbal Guys Development Team. All rights reserved.
#
############################################################################
px4_add_module(
	MODULE modules__open_gimbal
	MAIN open_gimbal
	COMPILE_FLAGS
	SRCS
		open_gimbal.cpp
	)

