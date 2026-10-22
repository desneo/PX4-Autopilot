############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__simulation__dyt_udp_bridge
	MAIN dyt_udp_bridge
	SRCS
		DytUdpBridge.cpp
)
