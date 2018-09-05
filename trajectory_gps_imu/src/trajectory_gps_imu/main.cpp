#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <lib.h>

#define PI 3.141592653589793238462643383279502884
#define record_duration 200
using namespace std;
using namespace gtsam;

NonlinearFactorGraph graph;
Values initialEstimate;
long node_id = 0;
bool init_flag_imu = true;
bool init_flag_gps = true;
bool task_not_done_imu = true;
bool task_not_done_gps = true;
bool ros_OK = true;
double gps_start_time = 0;
double imu_start_time = 0;
double last_time = 0;
vector<double> start_position(2);

ofstream myfile1;
ofstream myfile2;
double pre_time;
long cnt_imu = 0;
long cnt_gps = 0;

class UnaryFactor: public NoiseModelFactor1<Pose2> {

  // The factor will hold a measurement consisting of an (X,Y) location
  // We could this with a Point2 but here we just use two doubles
  double mx_, my_;

public:
  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<UnaryFactor> shared_ptr;

  // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
    NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

  virtual ~UnaryFactor() {}

  // Using the NoiseModelFactor1 base class there are two functions that must be overridden.
  // The first is the 'evaluateError' function. This function implements the desired measurement
  // function, returning a vector of errors when evaluated at the provided variable value. It
  // must also calculate the Jacobians for this measurement function, if requested.
  Vector evaluateError(const Pose2& q, boost::optional<Matrix&> H = boost::none) const
  {
    // The measurement function for a GPS-like measurement is simple:
    // error_x = pose.x - measurement.x
    // error_y = pose.y - measurement.y
    // Consequently, the Jacobians are:
    // [ derror_x/dx  derror_x/dy  derror_x/dtheta ] = [1 0 0]
    // [ derror_y/dx  derror_y/dy  derror_y/dtheta ] = [0 1 0]
    if (H) (*H) = (Matrix(2,3) << 1.0,0.0,0.0, 0.0,1.0,0.0).finished();
    return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
  }

  // The second is a 'clone' function that allows the factor to be copied. Under most
  // circumstances, the following code that employs the default copy constructor should
  // work fine.
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }

  // Additionally, we encourage you the use of unit testing your custom factors,
  // (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
  // GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.

}; // UnaryFactor

void imu_callback(const sensor_msgs::ImuConstPtr imu_msg){

  ++cnt_imu;
  if( (cnt_imu % 2) != 0) return;

  double imu_time = imu_msg->header.stamp.toSec();

    if(init_flag_imu == true){
      //IMU write
      myfile1 << imu_time << " " << imu_time << " " << imu_msg->linear_acceleration.x << " " << imu_msg->linear_acceleration.y << " " << imu_msg->linear_acceleration.z << " " <<
               imu_msg->angular_velocity.x << " " << imu_msg->angular_velocity.y << " " << imu_msg->angular_velocity.z << endl;
      init_flag_imu = false;
      imu_start_time = imu_time;
      pre_time = imu_time;

      return;
  }

  double dt = imu_time - pre_time;
  pre_time = imu_time;

  myfile1 << imu_time << " " << dt << " " << imu_msg->linear_acceleration.x << " " << imu_msg->linear_acceleration.y << " " << imu_msg->linear_acceleration.z << " " <<
               imu_msg->angular_velocity.x << " " << imu_msg->angular_velocity.y << " " << imu_msg->angular_velocity.z << endl ;

  if(imu_time - imu_start_time > record_duration){
    task_not_done_imu = false;
    myfile1.close();
  }
  return;
}

void gps_callback(const sensor_msgs::NavSatFixConstPtr gps_msg){

  ++cnt_gps;
  if( (cnt_gps % 40) != 0) return;

  ROS_INFO("GPS");

  double gps_time = gps_msg->header.stamp.toSec();

  if(init_flag_gps == true){
      //IMU write
      init_flag_gps = false;
      gps_start_time = gps_time;
      pre_time = gps_time;

      return;
  }

  double x_lat = (gps_msg->latitude - start_position[0]) * 111.32*1000;
  double y_lon = (gps_msg->longitude - start_position[1]) * 40075 *1000 * cos( start_position[0]*PI/180 ) / 360;

  myfile2 << gps_time << "," << x_lat << "," << y_lon << "," << gps_msg->altitude << endl;

  if(gps_time - gps_start_time > record_duration){
    task_not_done_gps = false;
    myfile2.close();
  }
  return;
}

void gps_imu_handle(const sensor_msgs::NavSatFixConstPtr gps_msg, const sensor_msgs::ImuConstPtr imu_msg){

 //  	noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1)); // 10cm std on x,y

	// if(init_flag == true){
	// 	start_time = (gps_msg->header.stamp.toSec() + imu_msg->header.stamp.toSec())/2;
	// 	start_position[0] = gps_msg->latitude;
	// 	start_position[1] = gps_msg->longitude;
	// 	init_flag = false;

 // 		graph.emplace_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise);
 //  		initialEstimate.insert(1, Pose2(0.0, 0.0, 0.0));
 // 		node_id = 2;
	// 	return;
	// }

	// double time = (gps_msg->header.stamp.toSec() + imu_msg->header.stamp.toSec())/2 - start_time;
	// if(time > 200){
	// 	LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
	// 	Values result = optimizer.optimize();
	// 	// result.print("Final Result:\n");

	// 	for(auto x :result){
	// 		x.value.print("");
	// 	}

 // 		ros_OK = false;
	// 	return;
	// }
	// // double x_lat = getDistance(start_position[0], gps_msg->longitude, gps_msg->latitude, gps_msg->longitude) * (start_position[0]<gps_msg->latitude?1:-1);
	// // double y_lon = getDistance(gps_msg->latitude, start_position[1], gps_msg->latitude, gps_msg->longitude) * (start_position[1]<gps_msg->longitude?1:-1);
	// double x_lat = (gps_msg->latitude - start_position[0]) * 111.32*1000;
	// double y_lon = (gps_msg->longitude - start_position[1]) * 40075 *1000 * cos( start_position[0]*PI/180 ) / 360;

 //  	graph.emplace_shared<UnaryFactor>(node_id, x_lat, y_lon, unaryNoise);
 //  	initialEstimate.insert(node_id, Pose2(x_lat, y_lon, 0.0));

 //  	// cout << node_id << "th insert is done" << endl;
 //  	++node_id;
 //  	return;

  // ROS_INFO("OJBK");

  // double imu_time = imu_msg->header.stamp.toSec();
  // double gps_time = gps_msg->header.stamp.toSec();


  // if(init_flag == true){
  //     //IMU write
  //     myfile1 << imu_time << " " << imu_time << " " << imu_msg->linear_acceleration.x << " " << imu_msg->linear_acceleration.y << " " << imu_msg->linear_acceleration.z << " " <<
  //              imu_msg->angular_velocity.x << " " << imu_msg->angular_velocity.y << " " << imu_msg->angular_velocity.z << endl;
  //     init_flag = false;
  //     start_time = imu_time;
  //     pre_time = imu_time;

  //     //GPS write
  //     start_position[0] = gps_msg->latitude;
  //     start_position[1] = gps_msg->longitude;
  //     myfile2 << gps_time << "," << 0 << "," << 0 << "," << gps_msg->altitude << endl;;

  //     return;
  // }

  // double dt = imu_time - pre_time;
  // pre_time = imu_time;

  // myfile1 << imu_time << " " << dt << " " << imu_msg->linear_acceleration.x << " " << imu_msg->linear_acceleration.y << " " << imu_msg->linear_acceleration.z << " " <<
  //              imu_msg->angular_velocity.x << " " << imu_msg->angular_velocity.y << " " << imu_msg->angular_velocity.z << endl ;

  // double x_lat = (gps_msg->latitude - start_position[0]) * 111.32*1000;
  // double y_lon = (gps_msg->longitude - start_position[1]) * 40075 *1000 * cos( start_position[0]*PI/180 ) / 360;

  // myfile2 << gps_time << "," << x_lat << "," << y_lon << "," << gps_msg->altitude << endl;

  // cout << imu_time - start_time << endl;
  // if(imu_time - start_time > 1){
  //   task_not_done = false;
  //   myfile1.close();
  //   myfile2.close();
  // }



  // return;
}

int main(int argc, char** argv){

  myfile1.open ("imu.txt");
  myfile2.open ("gps.txt");

  myfile1.precision(30);
  myfile2.precision(30);

  myfile1 << "Time dt accelX accelY accelZ omegaX omegaY omegaZ" << endl;
  myfile2 << "Time,X,Y,Z" << endl;

	ros::init(argc, argv, "trajectory_gps_imu");

	ros::NodeHandle nh;

  ros::Subscriber imu = nh.subscribe("an_device/Imu", 10, imu_callback);
  ros::Subscriber gps = nh.subscribe("an_device/NavSatFix", 10, gps_callback);

	// message_filters::Subscriber<sensor_msgs::NavSatFix> sub_gps(nh, "an_device/NavSatFix", 10);
	// message_filters::Subscriber<sensor_msgs::Imu> sub_imu(nh, "an_device/Imu", 10);

	// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::Imu> MySyncPolicy;
	// message_filters::Synchronizer<MySyncPolicy> gps_imu(MySyncPolicy(10), sub_gps, sub_imu);
	// gps_imu.registerCallback(boost::bind(&gps_imu_handle, _1, _2));

	while(ros_OK && (task_not_done_gps || task_not_done_imu) )
		ros::spinOnce();

}
