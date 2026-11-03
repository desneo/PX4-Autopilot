############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__simulation__aisheng_udp_bridge
	MAIN aisheng_udp_bridge
	SRCS
		AishengUdpBridge.cpp
)
