#include "reactive_velodyne_alg_node.h"

ReactiveVelodyneAlgNode::ReactiveVelodyneAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<ReactiveVelodyneAlgorithm>()
{
  //init class attributes if necessary
  flag_new_velodyne_data_ = false;

  z_threshold_ = MIN_OBSTACLE_HEIGHT_;

  abs_lateral_safety_margin_ = 0.1;

  safety_width_ = VEHICLE_WIDTH_ + (2.0 * abs_lateral_safety_margin_);

  euclidean_association_threshold_ = 0.10;
  min_obstacle_radius_ = 0.03;

  closest_front_obstacle_point_ = OUT_OF_RANGE_;
  closest_back_obstacle_point_ = OUT_OF_RANGE_;

  steering_angle_ = 0.0;

  this->loop_rate_ = 500;

  // [init publishers]
  this->front_obstacle_distance_publisher_ = this->public_node_handle_.advertise < std_msgs::Float32
      > ("front_obstacle_distance", 1);

  this->back_obstacle_distance_publisher_ = this->public_node_handle_.advertise < std_msgs::Float32
      > ("back_obstacle_distance", 1);

  this->pointcloud_publisher_ = this->public_node_handle_.advertise < sensor_msgs::PointCloud2 > ("pointcloud", 1);

  // [init subscribers]
  this->velodyne_subscriber_ = this->public_node_handle_.subscribe("/velodyne_points", 1,
                                                                   &ReactiveVelodyneAlgNode::velodyneCB, this);

  this->ackermann_subscriber_ = this->public_node_handle_.subscribe("/estimated_ackermann_state", 1,
                                                                    &ReactiveVelodyneAlgNode::estimatedAckermannStateCB,
                                                                    this);

  pthread_mutex_init(&this->velodyne_mutex_, NULL);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

ReactiveVelodyneAlgNode::~ReactiveVelodyneAlgNode(void)
{
  // [free dynamic memory]
}

void ReactiveVelodyneAlgNode::mainNodeThread(void)
{
  this->velodyne_mutex_enter();
  if (flag_new_velodyne_data_)
  {
    // [fill msg structures]
    flag_new_velodyne_data_ = false;
    local_copy_of_input_cloud_ = input_cloud_;
    this->velodyne_mutex_exit();

    // Do some cool stuff

    // [publish messages]
    front_obstacle_distance_msg_.data = closest_front_obstacle_point_ - DISTANCE_FROM_SENSOR_TO_FRONT_;
    this->front_obstacle_distance_publisher_.publish(this->front_obstacle_distance_msg_);

    back_obstacle_distance_msg_.data = closest_back_obstacle_point_ - DISTANCE_FROM_SENSOR_TO_BACK_;
    this->back_obstacle_distance_publisher_.publish(this->back_obstacle_distance_msg_);

    obstacle_points_.header.frame_id = local_copy_of_input_cloud_.header.frame_id;
    obstacle_points_.header.stamp = local_copy_of_input_cloud_.header.stamp;
    pointcloud_msg_ = obstacle_points_;
    this->pointcloud_publisher_.publish(this->pointcloud_msg_);

  }
  else
  {
    this->velodyne_mutex_exit();
  }
}

void ReactiveVelodyneAlgNode::velodyneCB(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  this->velodyne_mutex_enter();

  input_cloud_ = *msg;

  //DEBUG!!
  //std::cout << "Velodyne scan received!" << std::endl;
  if (msg == NULL)
    std::cout << std::endl << "Null pointer!!! in function velodyneCB!";

  flag_new_velodyne_data_ = true;

  this->velodyne_mutex_exit();
}

void ReactiveVelodyneAlgNode::estimatedAckermannStateCB(
    const ackermann_msgs::AckermannDriveStamped& estimated_ackermann_state_msg)
{
  this->velodyne_mutex_enter();

  //DEBUG!!
  //std::cout << "Ackermann state received!" << std::endl;

  steering_angle_ = estimated_ackermann_state_msg.drive.steering_angle;

  this->velodyne_mutex_exit();
}

void ReactiveVelodyneAlgNode::velodyne_mutex_enter(void)
{
  pthread_mutex_lock(&this->velodyne_mutex_);
}

void ReactiveVelodyneAlgNode::velodyne_mutex_exit(void)
{
  pthread_mutex_unlock(&this->velodyne_mutex_);
}

void ReactiveVelodyneAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void ReactiveVelodyneAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < ReactiveVelodyneAlgNode > (argc, argv, "reactive_velodyne_alg_node");
}
