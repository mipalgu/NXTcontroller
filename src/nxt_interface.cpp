/**
 * \file  nxt_interface.cpp
 *  nxt_interface.cpp
 *  Created by
 *  \author Vlad Estivill-Castro
 *  \date 14/10/2014.
 */

/*
 *
 * This is module acts as a bridge between ROS and a LEGO Mindstorm.
 * Sensors can be commanded to post updates and actuators can be
 * controlled.
 *
 *  Copyright 2012 Vlad Estivill-Castro. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. All advertising materials mentioning features or use of this
 *    software must display the following acknowledgement:
 *
 *        This product includes software developed by Rene Hexel.
 *
 * 4. Neither the name of the author nor the names of contributors
 *    may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * -----------------------------------------------------------------------
 * This program is free software; you can redistribute it and/or
 * modify it under the above terms or under the terms of the GNU
 * General Public License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see http://www.gnu.org/licenses/
 * or write to the Free Software Foundation, Inc., 51 Franklin Street,
 * Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include "ros_webots_epuck_nxt_differential_robot/nxt_interface.h"

           ///< constructor
NXT_interface::NXT_interface()
    { banner();

	r2d2::USBBrickManager usbm;

	brick = usbm.list()->at(0);
	
        nxt = brick->configure(r2d2::SensorType::TOUCH_SENSOR,
                                        r2d2::SensorType::TOUCH_SENSOR,
                                        r2d2::SensorType::NULL_SENSOR,
                                        r2d2::SensorType::NULL_SENSOR,
                                        r2d2::MotorType::STANDARD_MOTOR,
                                        r2d2::MotorType::STANDARD_MOTOR,
                                        r2d2::MotorType::STANDARD_MOTOR);
        if (nxt != nullptr) {
		/* check the connections of the NXT you are using */
        	sensor_right = nxt->sensorPort(r2d2::SensorPort::IN_1);
        	sensor_left = nxt->sensorPort(r2d2::SensorPort::IN_2);
        	motor_right = nxt->motorPort(r2d2::MotorPort::OUT_B);
        	motor_left = nxt->motorPort(r2d2::MotorPort::OUT_C);
		// initially sensors are off
		status_sensor_left = false;
		status_sensor_right = false;
               ROS_INFO("Connection established" );
	}
	else
               ROS_INFO("ERROR: Conenction failed" );
    }

void NXT_interface ::  run(int argc, char **argv)
{
        if (nxt != nullptr) {
                ros::init(argc, argv, "nxt_driver");
                ros::NodeHandle n;

                ros::Subscriber subRobot = n.subscribe("robot", 1000, & NXT_interface::robotCallback,this);

                ros::ServiceServer serviceStatusButton = n.advertiseService("buttonstatus",  & NXT_interface::value_buttonCallback,this);
		ROS_INFO("Service ready");
                ros::spin();

		std::cerr<< "This EXITING sometimes does not happen when roscore goes down" << std::endl;
        }
}

        /// call-back method button status/value
bool NXT_interface :: value_buttonCallback(
             ros_webots_epuck_nxt_differential_robot::RbuttonStatus::Request & req,
             ros_webots_epuck_nxt_differential_robot::RbuttonStatus::Response& res)
{
	if (status_sensor_right)
	{ res.right_pressed=sensor_right->getValue();
	}
	if (status_sensor_left)
	{ res.left_pressed=(true==sensor_left->getValue());
	}

	if (! (status_sensor_left || status_sensor_right))
   	    {  
               ROS_INFO("service invoked with both SENSOR OFF:" );
               ROS_INFO("FALSE exit:" );
	       return false;
	    }
	else 
	{ 
		return true;
	}
}

        // call-back method robot_control
void NXT_interface ::  robotCallback(const ros_webots_epuck_nxt_differential_robot::Rrobot::ConstPtr& msg) {
	int leftPower=msg->leftMotor.power; 
	int rightPower=msg->rightMotor.power; 

	ROS_INFO("Setting Motors Left: [%d] Right: [%d]", leftPower,rightPower);
	if ( (leftPower) && (100>= leftPower) )
		motor_left->setForward(leftPower);
	else 	{ // negative values should be back
			leftPower= - leftPower;
			if ( (leftPower) && (100>= leftPower) )
				motor_left->setReverse(leftPower);
			else // stop with power ==0
			motor_left->stop(false); 
		}

	if ( (rightPower) && (100>= rightPower) )
		motor_right->setForward(rightPower);
	else
		{ 	rightPower= -rightPower;
			if ( (rightPower) && (100>= rightPower) )
				motor_right->setForward(rightPower);
			else
				motor_right->stop(false); 
		}

	status_sensor_left =static_cast<bool>(msg->leftButtonSwitch.on);
	status_sensor_right =static_cast<bool>(msg->rightButtonSwitch.on);
	ROS_INFO("Switch sensors Left: [%s] Right: [%s]", status_sensor_left ? "ON" : "OFF",  status_sensor_right? "ON" :"OFF");
}

