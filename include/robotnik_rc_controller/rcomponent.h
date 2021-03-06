/*! \class RComponent
 *  \file RComponent.h
 *	\author Robotnik Automation S.L.L
 *	\version 0.1.0
 *	\date 2014
 *  \brief Class to define a standard and shared structure (attributes & methods) for all the components
 * 
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef __CURTISCONTROLLER_H
	#define __CURTISCONTROLLER_H

#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <vector>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <robotnik_msgs/State.h>

//! Size of string for logging
#define DEFAULT_THREAD_DESIRED_HZ	200.0

using namespace std;

//! Defines return values for methods and functions
enum ReturnValue{
	OK = 0,
	INITIALIZED,
	THREAD_RUNNING,
	ERROR = -1,
	NOT_INITIALIZED = -2,
	THREAD_NOT_RUNNING = -3,
	COM_ERROR = -4,
	NOT_ERROR = -5
};


//! Class Rcomponent
class RComponent{
	protected:
		//! Controls if has been initialized succesfully
		bool initialized, ros_initialized;
		//! Controls the execution of the RComponent's thread
		bool running;
		
		//! State of the RComponent
		int state;
		//! State before
		int previous_state;
		//!	Saves the name of the component
		string component_name;
		//! ROS node handle
		ros::NodeHandle nh_;
		//! Private ROS node handle
		ros::NodeHandle pnh_;
		//! Desired loop frequency
		double desired_freq_, real_freq;
		
		//! Publish the component state
		ros::Publisher state_publisher;
		
	public:
		//! Public constructor
		RComponent(ros::NodeHandle h);
		//! Public destructor
		virtual ~RComponent();
		
		//! Starts the control loop of the component and its subcomponents
		//! @return OK
		//! @return ERROR starting the thread
		//! @return RUNNING if it's already running
		//! @return NOT_INITIALIZED if it's not initialized
		virtual int start();
		//! Stops the main control loop of the component and its subcomponents
		//! @return OK
		//! @return ERROR if any error has been produced
		//! @return NOT_RUNNING if the main thread isn't running
		virtual int stop();
		//! Returns the general state of the RComponent
		int getState();
		//! Returns the general state of the RComponent as string
		char *getStateString();
		//! Returns the general state as string
		char *getStateString(int state);
		//! Method to get current update rate of the thread
		//! @return pthread_hz
		double getUpdateRate();
		
	protected:
		//! Configures and initializes the component
		//! @return OK
		//! @return INITIALIZED if the component is already intialized
		//! @return ERROR
		virtual int setup();
		//! Closes and frees the reserved resources
		//! @return OK
		//! @return ERROR if fails when closes the devices
		//! @return RUNNING if the component is running
		//! @return NOT_INITIALIZED if the component is not initialized
		virtual int shutdown();
		//! All core component functionality is contained in this thread.
		//!	All of the RComponent component state machine code can be found here.
		virtual void controlLoop();
		//! Actions performed on initial state
		virtual void initState();
		//! Actions performed on standby state
		virtual void standbyState();
		//! Actions performed on ready state
		virtual void readyState();
		//! Actions performed on the emergency state
		virtual void emergencyState();
		//! Actions performed on Failure state
		virtual void failureState();
		//! Actions performed on Shudown state
		virtual void shutdownState();
		//! Actions performed in all states
		virtual void allState();
		//! Switches between states
		virtual void switchToState(int new_state);
		//! Setups all the ROS' stuff
		virtual int rosSetup();
		//! Shutdowns all the ROS' stuff
		virtual int rosShutdown();
		//! Reads data a publish several info into different topics
		virtual void rosPublish();
		//! Reads params from params server
		virtual void rosReadParams();
		
};

#endif
