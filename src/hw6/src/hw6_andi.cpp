/*************************************************************/
/* 															 */
/* 			   CSE 276B: Human Robot Interaction 			 */
/*   Week 4, HW 6: Non-verbal sensing & multimodal control   */
/*                        Andi Frank                         */
/* 															 */
/*************************************************************/


/*************************************************************/
/*							SETUP							 */
/*************************************************************/
// #include <kobuki_msgs/BumperEvent.h> 
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <cmvision/Blobs.h>
#include <stdio.h>
#include <vector>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include <math.h>

//		MACROS

// Center of the screen.
#define SCREEN_CENTER 320
#define CENTER_THRESHOLD 10

#define GOAL_Z 0.2 		// distance at which we consider the target acquired (units unidentified)
#define OBS_Z 0.55 		// closet distance the bot will get to an obstacle 
#define ZTHRESH 0.7		// distance at which points are considered a threat

// DetectedObject color detection.
#define GOAL_R 120
#define GOAL_G 199
#define GOAL_B 147
#define COLOR_THRESHOLD 0

// PID controller variables.
#define KP_Z 0.001
#define KP 0.001
#define KD 0.0000001
#define KI 0.00000001
#define IMAX 7200000
#define IMIN 0
#define ERROR_THRESHOLD 0 //0.2
#define MAX_ERROR 100
double last_error = 0;
double i_error = 0;

// PointCloud type definition
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// DetectedObject data
struct DetectedObject {
	double x;
	double z;	
};
DetectedObject target;
DetectedObject obstacle;

// Velocity publisher
ros::Publisher vel_pub;

// not entirely sure what this is for?
bool got_goal_blobs = false;


/*************************************************************/
/*						 	   FSM						  	 */
/*************************************************************/

// States.
enum State {
	NO_TARGET,
	TARGET_UNCENTERED,
	TARGET_CENTERED,
	TARGET_ACQUIRED,
	PROXIMATE_OBSTACLE
};
State state = NO_TARGET;

void update_state(){
	// Update state.
	 if ( obstacle.z < OBS_Z ) 			{	state = PROXIMATE_OBSTACLE; }  // If an obstacle is too close, avoid.
	 else 
	 if ( abs(target.x - SCREEN_CENTER) < CENTER_THRESHOLD) 		// Otherwise, if within threshold of center...
	 	if ( target.z <= GOAL_Z ) 	{ 	state = TARGET_ACQUIRED; 	}		// AND close enough, target acquired!
	 	else 						{ 	state = TARGET_CENTERED; 	}		// BUT not close enough, target centered.
	 else 							{	state = TARGET_UNCENTERED;	}	// Otherwise, target not centered.
}

/////////////////////////////////////////////////////////////
// 						ACTIONS

void avoid(){
	ROS_INFO("Avoiding...");
	// turn away from object for 2 seconds, and move forward for 5 seconds
	geometry_msgs::Twist cmd;
	cmd.angular.z = obstacle.x - SCREEN_CENTER;
	vel_pub.publish(cmd);
	ros::Duration(2.0).sleep();

	// move forward
	cmd.angular.z = 0;
	cmd.linear.x = 0.5;
	vel_pub.publish(cmd);
	ros::Duration(5.0).sleep();

	update_state();
}

void wander(){
	ROS_INFO("Wandering...");

	geometry_msgs::Twist cmd;
    cmd.linear.x = 0.4;
    cmd.angular.z = 0.1;
    vel_pub.publish(cmd);
}


/************************************************************
 * Function Name: 	aim()
 * Inputs:			None.
 * Outputs:			double, control signal to center the target.
 *
 * Description:	@brief: Calculates control signal to center
 *					target horizontally.
 * 				Uses a PID controller to track error and
 * 					calculate a control signal to center the 
 * 					x coordinate of the target.
 ***********************************************************/
void aim() {
	ROS_INFO("Aiming...");

	// Calculate PID terms.
	double error = target.x - SCREEN_CENTER;
	if(isnan(error)) { error = MAX_ERROR; }
	double p_term = KP * error;
	double d_term = KD * (error - last_error);

	// Deal with integral windup.
	if 		( i_error > IMAX )		{	i_error = IMAX;	}
	else if ( i_error < IMIN )		{	i_error = IMIN;	}
	else 							{	i_error = i_error + error;	}
	double i_term = KI * i_error;
	last_error = error;

	// Calculate control signal.
	double ctrl_signal = p_term + d_term + i_term;

	// Only publish control signal if error is above threshold.
	geometry_msgs::Twist cmd;
    cmd.linear.x = KP_Z * (target.z - GOAL_Z);
	if ( abs(ctrl_signal) < ERROR_THRESHOLD ) {		cmd.angular.z = 0.0;			}
	else 									  {		cmd.angular.z = ctrl_signal;	}
	vel_pub.publish(cmd);
}


/************************************************************
 * Function Name: 	approach()
 * Inputs:			None.
 * Outputs:			void.
 *
 * Description:	@brief: Moves forward toward centered target.
 * 				Moves the turtlebot directly forward, assuming
 * 					that the target is already centered from
 * 					aim(). Main loop should handle re-centering
 *					if necessary.
 ***********************************************************/
void approach() {
	ROS_INFO("Approaching target...");
	geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
    cmd->linear.x = (target.z - GOAL_Z) * KP_Z;
    vel_pub.publish(cmd);
}

/************************************************************
 * Function Name: 	halt()
 * Inputs:			None.
 * Outputs:			void.
 *
 * Description:	@brief: Stops the turtlebot.
 *				Stops all motor functions of the robot. Should
 *					be called when the target is acquired.
 ***********************************************************/
void halt() {
	ROS_INFO("Halting...");
	// Stop the robot.
	vel_pub.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
}


/*************************************************************/
/*		     	     HARDWARE COMMUNICATION				  	 */
/*************************************************************/

// 							Callbacks

/************************************************************
 * Function Name: 	blobs_callback()
 * Inputs:			const cmvision::Blobs&, pointer to Blobs 
 * 				 		object from the /blobs topic.
 * Outputs:			void.
 *
 * Description:	@brief: Finds x centroid of target color.
 * 				Function reads in blobs_in, checks if the
 * 					blob is the target color, and stores the
 * 					x coordinate of the centroid of the target
 * 					color. State variable is updated to
 *					TARGET_CENTERED or TARGET_UNCENTERED if
 *					needed.
 ***********************************************************/

void blob_callback(const cmvision::Blobs& blobs_in) {
	double target_sum_x = 0;
	int i;
	for (i = 0; i < blobs_in.blob_count; i++) {
		// if the color is within threshold units of the goal color
		if (	abs( blobs_in.blobs[i].red 		-	GOAL_R ) 	< COLOR_THRESHOLD &&
				abs( blobs_in.blobs[i].green	-	GOAL_G )	< COLOR_THRESHOLD &&
				abs( blobs_in.blobs[i].blue		-	GOAL_B )	< COLOR_THRESHOLD )
		{
			 target_sum_x += blobs_in.blobs[i].x; //stores the sum coordinates of the target blobs
		}
	}
	// Find average horizontal position of target color centroid and update target.x.
	 target.x = target_sum_x/i;

	 // Update state.
	 update_state();

}

/******************************************************************
 * Function Name: 	cloud_callback
 * Inputs:			const PointCloud::ConstPtr&, pointer to 
 *						point cloud object.
 * Outputs:			void
 *
 * Description:		@brief: Finds minimum z distance of point cloud.
 *					Finds the closest z distance of the point cloud 
 *						in the bot's FOV. Only considers points below
 *						ZTHRESH. For obstacle avoidance.
 ****************************************************************/
void cloud_callback(const PointCloud::ConstPtr& cloud_msg)
{
	//ROS_INFO("PointCloud callback.");

	// Reset variables.
	obstacle.z = 1e6;
	obstacle.x = 0;
	double count = 0;

	// Iterate through points.
	for ( int j = 0; j < 240; j++ ){
		for ( int i = 0; i < 640; i++ ) {
			const pcl::PointXYZ& pt = cloud_msg->points[ 640 * ( 180 + j ) + i];
			// Only pay attention to points close enough to bot.
			if( pt.z < ZTHRESH){
				obstacle.x += obstacle.x;
				// Of these, find closest distance to obstacle.
				if ( pt.z < obstacle.z ) { obstacle.z = pt.z; }
			}
		} 
	}
	obstacle.x /= count;
	update_state();
}


/*************************************************************/
/*						    MAIN			  				 */
/*************************************************************/

int main (int argc, char *argv[]){
	// Initiate ROS processes.
	ros::init(argc, argv, "hw6");
	ros::NodeHandle nh;

	// Create subscriber and publisher for relevant topics.
    ros::Subscriber pc_sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, cloud_callback);  
    ros::Subscriber blob_sub = nh.subscribe("/blobs", 100, blob_callback);
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);
	
	// Set the loop frequency in Hz.
    ros::Rate fs(10);

    // Denote that setup is complete.
    ROS_INFO("Program started.");

	while(ros::ok()){
		// control flow
		if 		( state == PROXIMATE_OBSTACLE ) { 	avoid();	}
		else if ( state == TARGET_ACQUIRED ) 	{ 	halt();		}
		else if ( state == TARGET_CENTERED ) 	{ 	approach(); }
		else if ( state == TARGET_UNCENTERED ) 	{ 	aim();		}
		else 									{	wander();	}

		// Spin.
		ros::spinOnce();
		// Sleep to sample rate.
		fs.sleep();
	}
}

