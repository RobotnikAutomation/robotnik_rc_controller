/*! \class robotnik_rc_controller
 *  \file robotnik_rc_controller.cpp
 *	\author Robotnik Automation S.L.L
 *	\version 0.1.0
 *	\date 2017
 *  \brief Component that subscribes to mavros RC/IN topic and publishes cmd_vel references for the robot traction 
 *                 depending on take-over switch
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

#include <robotnik_rc_controller/rcomponent.h>
#include <mavros_msgs/RCIn.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#define MAX_VAL 			2006.0
#define MIN_VAL 			982.0
#define CEN_VAL 			1494.0
#define MAX_CHANNELS 		32

#define DEFAULT_CH_LEVEL 		1
#define DEFAULT_CH_W  			2
#define DEFAULT_CH_X  			3
#define DEFAULT_CH_Y  			4
#define DEFAULT_CH_TAKE_OVER 	6
#define DEFAULT_DEAD_ZONE   	0.05

#define MAX_LINEAR_SPEED    	3.0    // m/s
#define MAX_ANGULAR_SPEED  		6.28   // rad/s

using namespace std;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown;

//! Class robotnik_rc_controller
class robotnik_rc_controller: public RComponent{
	private:
    ros::Publisher cmd_vel_pub_;
    ros::Publisher take_over_pub_; 
    ros::Subscriber rc_in_sub_;
    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;
    string topic_cmd_vel_;
    string topic_rc_in_;
    double dead_zone_;
    bool take_over_;
    // to publish
    double ch_x_, ch_y_, ch_w_;
    double scale_x_, scale_y_, scale_w_;
      
	public:	
	// Constructor
    robotnik_rc_controller(ros::NodeHandle h):node_handle_(h), private_node_handle_("~"),RComponent(h){
		}

	protected:	
	// Inherits from RComponent
	int rosSetup(){
        ROS_INFO("ROS SETUP");
		if(RComponent::rosSetup() == OK){
          private_node_handle_.param("topic_cmd_vel", topic_cmd_vel_, string("/cmd_vel")); // be sure to manage correct twist priorities
          private_node_handle_.param("topic_rc_in", topic_rc_in_, string("/mavros/rc/in")); 
          private_node_handle_.param("dead_zone", dead_zone_, 0.05); 
          
          cmd_vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>(topic_cmd_vel_.c_str(), 10);         
          take_over_pub_ = node_handle_.advertise<std_msgs::Bool>("take_over", 50);
          rc_in_sub_ = node_handle_.subscribe<mavros_msgs::RCIn>(topic_rc_in_.c_str(), 1, &robotnik_rc_controller::rcInCallback, this);         
          // ROS_INFO("robotnik_rc_controller publish topic: %s",topic_cmd_vel_.c_str());
		  }                
        ch_x_ = 0.0;
        ch_y_ = 0.0;
        ch_w_ = 0.0; 
        take_over_ = false;		  
        scale_x_ = MAX_LINEAR_SPEED;
        scale_y_ = MAX_LINEAR_SPEED;
        scale_w_ = MAX_ANGULAR_SPEED;
	    }
    
    // Inherits from RComponent
	int rosShutdown(){
		if(RComponent::rosShutdown() == OK){
			ROS_INFO("rosShutdown");			
		    }
        }

	// Callback
	/*!     \fn void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
			* Callback - RC input references 
	*/
	void rcInCallback(const mavros_msgs::RCIn::ConstPtr& rc_in) {
		
		uint16_t channels[MAX_CHANNELS];
		
		
		// TBD : additional safety can be added by processing rssi that is in the RCIn msg.
		double range = MAX_VAL - MIN_VAL;
		/* int i=0; 
		 * for(std::vector<int>::const_iterator it = rc_in->channels.begin(); it != rc_in->channels.end(); ++it) {
			channels[i] = *it;
			if (channels[i] > MAX_VAL) channels[i] = MAX_VAL;
			if (channels[i] < MIN_VAL) channels[i] = MIN_VAL;
			i++;
			}*/

		if (rc_in->channels.size()<6) {
			ROS_ERROR("rcInCallback - incomplete message received");
			return;
			}

		for (int i=0; i<rc_in->channels.size(); i++) 
			  channels[i] = rc_in->channels[i];					
		
		// Convert the read channels to system values
		// Num on sender alu frame -> ci
		/* 
		t2 -> c1 (down: 982 center 1492, up: 2006)
		t1 -> c4 (left: 982, center: 1494 right: 2006)
		t3 -> c3 (down: 982 center 1494, up: 2006)
		t4 -> c2 (left: 982, center: 1494 right: 2006)
		6 -> c5, 3 positions: down : 2006, center: 1494, up: 2006
		2 -> c6, 2 positions: down : 2006, up: 982 
		8 -> c7, 3 positions: down : 2006, center: 1494, up: 2006
		9 -> c8, 3 positions: down : 2006, center: 1494, up: 2006
		5 -> c9, center 982, rotating left (z+): 982, rotating right (z-): incremental up to 1494
		1 -> c10, center 2006, rotating left (z+): decremental up to 1494, rotating right (z-): 2006
		7 -> c11, 3 positions: down : 2006, center: 1494, up: 2006
		8 -> c12, 3 positions: down : 2006, center: 1494, up: 2006
		4 -> c13, 2 positions (button up default): 982 , down: 2006
		10 -> c14, 3 positions: down : 2006, center: 1494, up: 2006
		S1 -> c15, center 982, up: incremental up to 1494, down: 982
		S2 -> c16, center 2006, up: 2006, down: decremental up to 1508 */
		
		double level = (MAX_VAL - (double) channels[DEFAULT_CH_LEVEL]) / range;  			// conversions of level - 0.0 ... 1.0	
		double x = ((double) channels[DEFAULT_CH_X] - CEN_VAL) / (range/2.0);       		// conversion of speed  982 (-1.0) up 2006.0 (+1.0)	
		if ((x>-dead_zone_)&&(x<dead_zone_)) x=0.0;
		ch_x_ = x * level * scale_x_;
		double y = -1.0 * ((double) channels[DEFAULT_CH_Y] - CEN_VAL) / (range/2.0);
		if ((y>-dead_zone_)&&(y<dead_zone_)) y=0.0;
		ch_y_ = y * level * scale_y_;		
		double w = -1.0 * ((double) channels[DEFAULT_CH_W] - CEN_VAL) / (range/2.0);
		if ((w>-dead_zone_)&&(w<dead_zone_)) y=0.0;
		ch_w_ = w * level * scale_w_;
		if (channels[DEFAULT_CH_TAKE_OVER] > CEN_VAL) take_over_ = false;	// channel with 2 positions
		else take_over_ = true;
		}


    void readyState(){
        RComponent::readyState();            
        geometry_msgs::Twist cmd_vel;
        
        if (take_over_) {
			cmd_vel.linear.x = ch_x_;
			cmd_vel.linear.y = ch_y_; 
			cmd_vel.linear.z = 0.0;
			cmd_vel.angular.x = 0.0;
			cmd_vel.angular.y = 0.0;
			cmd_vel.angular.z = ch_w_;
			cmd_vel_pub_.publish(cmd_vel);        
			}
			
		std_msgs::Bool msg;
		msg.data = take_over_;
		take_over_pub_.publish(msg);		
	    }

    void initState(){
        RComponent::initState();
        rosSetup();
        switchToState(robotnik_msgs::State::READY_STATE);
		}

};

// MAIN
int main(int argc, char** argv){
    ros::init(argc, argv, "robotnik_rc_controller", ros::init_options::NoSigintHandler);    
    ros::NodeHandle n;
    robotnik_rc_controller proc(n);	
    proc.start();
	return (0);
}
