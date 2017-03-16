/*
 * pilot.cpp
 *
 *  Created on: Mar 16, 2017
 *      Author: martin
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <xbee.h>

using namespace ros;

struct xbee *xbee;
struct xbee_con *con;

void vel_cb(const geometry_msgs::Twist vels)
{
	ROS_INFO("Last cmd_vel x val: %f\n", vels.linear.x);

	unsigned char vel_bytes[4];
	vel_bytes[0] = 128 + vels.linear.x * 127;
	vel_bytes[1] = 128 + vels.linear.x * 127;
	vel_bytes[2] = 128 - vels.linear.x * 127;
	vel_bytes[3] = 128 - vels.linear.x * 127;
	xbee_connTx(con, NULL, vel_bytes, 4);
}

int main(int argc, char ** argv)
{
	// Init xbee and connection
	xbee_err ret;
	if ((ret = xbee_setup(&xbee, "xbee1", "/dev/ttyUSB1", 57600)) != XBEE_ENONE) {
		printf("ret: %d (%s)\n", ret, xbee_errorToStr(ret));
		return ret;
	}

	struct xbee_conAddress address;
	memset(&address, 0, sizeof(address));
	address.addr16_enabled = 1;
	address.addr16[0] = 0x22;
	address.addr16[1] = 0x22;
	if ((ret = xbee_conNew(xbee, &con, "16-bit Data", &address)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conNew() returned: %d (%s)", ret, xbee_errorToStr(ret));
		return ret;
	}

	init(argc,argv, "pilot");

	NodeHandle n("~");

	Subscriber sub = n.subscribe("cmd_vel", 1, vel_cb);

	spin();

	return 0;
}


