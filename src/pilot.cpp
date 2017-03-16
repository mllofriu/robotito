/*
 * pilot.cpp
 *
 *  Created on: Mar 16, 2017
 *      Author: martin
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <semaphore.h>

#define LOOP_RATE 10

using namespace ros;

// Mutex to protect the stored Twist msg
sem_t twist_sem;
geometry_msgs::Twist last_vel;

void vel_cb(const geometry_msgs::Twist msg){
	//ROS_INFO("Received vel msg");

	sem_wait(&twist_sem);
	last_vel = msg;
	sem_post(&twist_sem);
}

int main(int argc, char ** argv)
{
	if (sem_init(&twist_sem, 0, 1) == -1)
    {
		printf("sem_init: failed: %s\n", strerror(errno));
		return errno;
    }

	init(argc,argv, "pilot");

	NodeHandle n("~");

	Subscriber sub = n.subscribe("cmd_vel", 100, vel_cb);

	Rate loop_rate(LOOP_RATE);
	while (ok()){
		sem_wait(&twist_sem);
		ROS_INFO("Last cmd_vel x val: %f\n", last_vel.linear.x);
		sem_post(&twist_sem);

		spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


