/*
 * pilot.cpp
 *
 *  Created on: Mar 16, 2017
 *      Author: martin
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

#include <xbee.h>

#include <math.h>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <float.h>

using namespace ros;

struct xbee *xbee;
struct xbee_con *con;

double LF_offset = 60 * M_PI / 180; // Robot's x-Axis (front) is zero
double LB_offset = 120 * M_PI / 180; //170
double RF_offset = 300 * M_PI / 180; //45
double RB_offset = 240 * M_PI / 180; //315
const double wheel_radius = 33;

double MAX_ANGULAR_VEL = 127;
double MAX_LINEAR_VEL = 127;
double MAX_WHEEL_SPEED = 127;

unsigned char vel_bytes[3] = { 128, 128, 128 };

int seq = 0;

void vel_cb(const geometry_msgs::Twist vels) {
//	ROS_INFO("Last cmd_vel x val: %f\n", vels.linear.x);

	// Get vels
	double x_vel = vels.linear.x * MAX_LINEAR_VEL;
	double y_vel = vels.linear.y * MAX_LINEAR_VEL;
	double t_vel = vels.angular.z * MAX_ANGULAR_VEL;

	// Set vels to send
	vel_bytes[0] = 128 + x_vel;
	vel_bytes[1] = 128 + y_vel;
	vel_bytes[2] = 128 + t_vel;
}

void sensor_cb(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt,
		void **data) {
	ROS_INFO("Received pkt");
	if ((*pkt)->dataLen > 0) {
		unsigned char * data = (*pkt)->data;
		for (int i = 0; i < 2; i += 2) {
			unsigned short val = (data[i] << 8) | data[i + 1];
			ROS_INFO("%u ", val);
		}
		ROS_INFO("\n");
	}
}

int main(int argc, char ** argv) {
	// Init xbee and connection
	xbee_err ret;
	if ((ret = xbee_setup(&xbee, "xbee1", "/dev/ttyUSB0", 57600))
			!= XBEE_ENONE) {
		printf("ret: %d (%s)\n", ret, xbee_errorToStr(ret));
		return ret;
	}

	struct xbee_conAddress address;
	memset(&address, 0, sizeof(address));
	address.addr16_enabled = 1;
	address.addr16[0] = 0x22;
	address.addr16[1] = 0x22;
	if ((ret = xbee_conNew(xbee, &con, "16-bit Data", &address))
			!= XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conNew() returned: %d (%s)", ret,
				xbee_errorToStr(ret));
		return ret;
	}
	struct xbee_conSettings settings;
	settings.disableAck = 1;
	settings.noBlock = 1;
	settings.noWaitForAck = 1;
	if ((ret = xbee_conSettings(con, &settings, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conSettings() returned: %d (%s)", ret,
				xbee_errorToStr(ret));
		return ret;
	}

	// Setup xbee callback for incoming information
//	if ((ret = xbee_conCallbackSet(con, sensor_cb, NULL)) != XBEE_ENONE) {
//		xbee_log(xbee, -1, "xbee_conCallbackSet() returned: %d", ret);
//		return ret;
//	}

	init(argc, argv, "pilot");

	NodeHandle n("~");

	Subscriber sub = n.subscribe("cmd_vel", 1, vel_cb);
	// Split range msgs in two because of bug in rviz
	Publisher pub1 = n.advertise<sensor_msgs::Range>("rangeRight", 12, false);
	Publisher pub2 = n.advertise<sensor_msgs::Range>("rangeLeft", 12, false);


	Rate rate(25);
	struct xbee_pkt * rxpkt;
	while (ok()) {
		// Receive sonar
		if (xbee_conRx(con, &rxpkt, NULL) == XBEE_ENONE) {
			if (rxpkt->dataLen > 0) {
				unsigned char * data = rxpkt->data;
				Time now = Time::now();
				for (unsigned int i = 0; i < rxpkt->dataLen; i += 2) {
					sensor_msgs::Range r;
					r.radiation_type = sensor_msgs::Range::INFRARED;
					r.field_of_view = M_PI/12;
					r.min_range = .05;
					r.max_range = .3;
					r.header.frame_id = "range" + boost::lexical_cast<std::string>(i/2);
					r.header.stamp = now;
					r.header.seq = seq++;
					// Conver to range
					unsigned int val = (data[i] << 8) | data[i + 1];

					float volt = (val/1024.0f) * 5;
					if (volt < .3)
						r.range = .3; //FLT_MAX;
					else if (volt < 1.8){
						// Inverse of distance using eq
						float distinv = 0.0758 * volt - 0.00265;
						float dist = 1 / distinv - 0.42;
						// return meters
						r.range = dist / 100;
					} else {

						float distinv = 0.1111 * volt - 0.07831;
						float dist = 1 / distinv - 0.42;
						// return meters
						r.range = dist / 100;
					}

					if (r.range > .3)
						r.range = .3;


					ROS_INFO("Range %f on sensor %d",r.range, i);

					if (i/2 >= 6)
						pub1.publish(r);
					else
						pub2.publish(r);
//					spinOnce();
				}
			}
			xbee_pktFree(rxpkt);
		}


		// Send controls
		xbee_connTx(con, NULL, vel_bytes, 3);

		spinOnce();
		rate.sleep();
	}

	return 0;
}

