############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__simulation__zhongke_udp_bridge
	MAIN zhongke_udp_bridge
	SRCS
		ZhongkeUdpBridge.cpp
)
