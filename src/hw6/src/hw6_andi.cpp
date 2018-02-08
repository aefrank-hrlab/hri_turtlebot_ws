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

// Center of the screen.
#define SCREEN_CENTER 320
#define CENTER_THRESHOLD 10

#define GOAL_Z 0.5 // distance at which we consider the target acquired (units unidentified)

// Target color detection.
#define GOAL_R 0
#define GOAL_G 150
#define GOAL_B 255
#define COLOR_THRESHOLD 30

// PID controller variables.
#define KP 0.005
#define KD 0.0000001
#define KI 0.00000001
#define IMAX 7200000
#define IMIN 0
#define ERROR_THRESHOLD 30

double last_error = 0;
double i_error = 0;

// PointCloud type definition
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Target data
struct Target {
	double x;
	double z;	
};
Target target;

// Velocity publisher.
ros::Publisher vel_pub;

// not entirely sure what this is for?
bool got_goal_blobs = false;


/*************************************************************/
/*						 	   FSM						  	 */
/*************************************************************/

// States.
enum Target_State{
	NO_TARGET,
	TARGET_UNCENTERED,
	TARGET_CENTERED,
	TARGET_ACQUIRED
};
Target_State state = NO_TARGET;

void update_state(){
	// Update state.
	 if ( abs(target.x - SCREEN_CENTER) < CENTER_THRESHOLD) 			// If within threshold of center...
	 	if ( target.z <= GOAL_Z ) 	{ 	state = TARGET_ACQUIRED; 	}		// AND close enough, target acquired!
	 	else 						{ 	state = TARGET_CENTERED; 	}		// BUT not close enough, target centered.
	 else 							{	state = TARGET_UNCENTERED;	}	// Otherwise, target not centered.
}

// ACTIONS
void wander(){
	ROS_INFO("Wandering...");
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
double aim() {
	ROS_INFO("Aiming...");
	// Calculate PID terms.
	double error = target.x - SCREEN_CENTER;
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

	// Only return control signal if error is above threshold.
	if ( abs(ctrl_signal) < ERROR_THRESHOLD )		{ return 0.0; }
	else											{ return ctrl_signal;	}
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
    cmd->linear.x = (target.z - GOAL_Z) * KP;
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
 *					needed. This processing is skipped if
 *					state == TARGET_ACQUIRED.
 ***********************************************************/
void blob_callback(const cmvision::Blobs& blobs_in) {
	ROS_INFO("blob_callback");
	// @TODO: Change this so only blob_callback or cloud_callback processes blobs at once
	//if (!got_goal_blobs){ 	// skip processing if target has already been acquired
		double target_sum_x = 0;
		int i;
		for (i = 0; i < blobs_in.blob_count; i++) {
			// if the color is within 10 units of the goal color
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

	//}
}

// @TODO: Examine cloud_callback() and make it your own.
//Threshhold for z distance
#define ZTHRESH 0.7

uint16_t goal_loc_x = 0; 	//global to store the x coordinate of the goal.
uint16_t goal_loc_y = 0; 	//global to store the y coordinate of the goal.
double goal_depth = 0; 		//global storing the depth of the goal blob
double z_min = 100;
std::vector<double> PCL_closest_points_z;
std::vector<double> PCL_closest_points_x;

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
	ROS_INFO("PointCloud callback.");
 //  // 					FILTER DATA
	// // Container for original & filtered data.
	// pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
	// pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	// pcl::PCLPointCloud2 cloud_filtered_pcl;

	// // Convert to PCL data type.
	// pcl_conversions::toPCL(*cloud_msg, *cloud);

	// // Perform the actual filtering.
	// pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	// sor.setInputCloud(cloudPtr);
	// sor.setLeafSize (0.1, 0.1, 0.1);
	// sor.filter(cloud_filtered_pcl);

	// // Convert to ROS data type.
	// sensor_msgs::PointCloud2 cloud_filtered;
	// pcl_conversions::fromPCL(cloud_filtered_pcl, cloud_filtered);

 //  // 				FIND CENTROID LOCATION
	// // Iterate through points.
	// for ( int j = 0; j < 240; j++ ){
	// 	for ( int i = 0; i < 640; i++ ) {
	// 		const pcl::PointXYZ& pt = cloud_filtered->points[ 640 * ( 180 + j ) + i];
	// 		// Only pay attention to points close enough to bot.
	// 		if( cloud_filtered.z < ZTHRESH){

	// 		}
	// 	} 
	// }
	// cout << "z: " << output.z << endl;

}


/*************************************************************/
/*						    MAIN			  				 */
/*************************************************************/

int main (int argc, char *argv[]){
	// Initiate ROS processes.
	ros::init(argc, argv, "hw6");
	ros::NodeHandle nh;

		// Local variables.
	geometry_msgs::Twist twist;

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
		if 		( state == TARGET_ACQUIRED ) 	{ 	halt();		}
		else if ( state == TARGET_CENTERED ) 	{ 	approach(); }
		else if ( state == TARGET_UNCENTERED ) 	{ 	aim();		}
		else 									{	wander();	}

		// Publish twist message.
		//vel_pub.publish(twist);
		// Spin.
		ros::spinOnce();
		// Sleep to sample rate.
		fs.sleep();
	}
}

