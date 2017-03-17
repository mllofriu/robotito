/*
 * pilot.cpp
 *
 *  Created on: Mar 16, 2017
 *      Author: martin
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <xbee.h>

#include <math.h>

using namespace ros;

struct xbee *xbee;
struct xbee_con *con;

double LF_offset = 60*M_PI/180; // Robot's x-Axis (front) is zero
double LB_offset = 120*M_PI/180; //170
double RF_offset =  300*M_PI/180; //45
double RB_offset = 240*M_PI/180; //315
const double wheel_radius = 33;

double MAX_ANGULAR_VEL = M_PI / 2;
double MAX_LINEAR_VEL = 64;
double MAX_WHEEL_SPEED = 64;

void vel_cb(const geometry_msgs::Twist vels)
{
	ROS_INFO("Last cmd_vel z val: %f\n", vels.angular.z);

	// Get vels
	double x_vel = vels.linear.x * MAX_LINEAR_VEL;
	double y_vel = vels.linear.y * MAX_LINEAR_VEL;
	double t_vel = vels.angular.z * MAX_ANGULAR_VEL;

	// Convert linear and angular to wheel vels
	double RF = (-sin(RF_offset) * x_vel + cos(RF_offset)*y_vel + wheel_radius*t_vel);
	double LF = (-sin(LF_offset) * x_vel + cos(LF_offset)*y_vel + wheel_radius*t_vel);
	double LB = (-sin(LB_offset) * x_vel + cos(LB_offset)*y_vel + wheel_radius*t_vel);
	double RB = (-sin(RB_offset) * x_vel + cos(RB_offset)*y_vel + wheel_radius*t_vel);

	ROS_INFO("RF: %f\n", RF);

	// Normalize to max speed
    if (abs(LF)>MAX_WHEEL_SPEED)
    {
        LB=(MAX_WHEEL_SPEED/abs(LF))*LB;
        RF=(MAX_WHEEL_SPEED/abs(LF))*RF;
        RB=(MAX_WHEEL_SPEED/abs(LF))*RB;
        LF=(MAX_WHEEL_SPEED/abs(LF))*LF;
    }
    if (abs(LB)>MAX_WHEEL_SPEED)
    {
        LF=(MAX_WHEEL_SPEED/abs(LB))*LF;
        RF=(MAX_WHEEL_SPEED/abs(LB))*RF;
        RB=(MAX_WHEEL_SPEED/abs(LB))*RB;
        LB=(MAX_WHEEL_SPEED/abs(LB))*LB;
    }
    if (abs(RF)>MAX_WHEEL_SPEED)
    {
        LF=(MAX_WHEEL_SPEED/abs(RF))*LF;
        LB=(MAX_WHEEL_SPEED/abs(RF))*LB;
        RB=(MAX_WHEEL_SPEED/abs(RF))*RB;
        RF=(MAX_WHEEL_SPEED/abs(RF))*RF;
    }
    if (abs(RB)>MAX_WHEEL_SPEED)
    {
        LF=(MAX_WHEEL_SPEED/abs(RB))*LF;
        LB=(MAX_WHEEL_SPEED/abs(RB))*LB;
        RF=(MAX_WHEEL_SPEED/abs(RB))*RF;
        RB=(MAX_WHEEL_SPEED/abs(RB))*RB;
    }

    ROS_INFO("RF Norm: %f\n", RF);

    // Set vels to send
    unsigned char vel_bytes[4];
	vel_bytes[0] = 128 + RB;
	vel_bytes[1] = 128 + RF;
	vel_bytes[2] = 128 + LB;
	vel_bytes[3] = 128 + LF;
	// Send the packet
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
	struct xbee_conSettings settings;
	settings.disableAck = 1;
	settings.noBlock = 1;
	settings.noWaitForAck = 1;
	if ((ret = xbee_conSettings(con, &settings, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conSettings() returned: %d (%s)", ret, xbee_errorToStr(ret));
		return ret;
	}

	init(argc,argv, "pilot");

	NodeHandle n("~");

	Subscriber sub = n.subscribe("cmd_vel", 1, vel_cb);

	spin();

	return 0;
}


