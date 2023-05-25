# ROS2 migration cheat sheet

## Suggested ROS2 workspace structure

```
.
├── config
│   └── config.yaml
├── include
│   └── my_pkg
│       └── node1.h
├── launch
│   ├── nodes.launch.py
│   └── nodes_with_cli_args.launch.py
├── msg
│   └── chatter.msg
├── README.md
├── src
│   └── my_pkg
│       └── node1.cpp
└── srv
    └── trigger.srv
```

## Custom message definitions
```C++
// ROS1
time stamp
Header header
// ROS2
builtin_interfaces/Time stamp
std_msgs/Header header
```

## Include message headers

```C++
// ROS1
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

// ROS2, lower case letters and .hpp file format, messages, services are namespaced ::msg
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
```

## Subscribe to a topic (PointCloud2)

```C++
// ROS1, from within node
auto nh = getNodeHandle(); 
ros::Subscrber cloud_sub = nh.subscribe( "/cloud_topic", queue_size, &cloud_callback, this );

// ROS2, from within node
// Note that Quality of Service (QoS) can be specified in more detail
std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> cloud_sub = \ 
    this->create_subscription<sensor_msgs::msg::PointCloud2>( "/cloud_topic", rclcpp::QoS( queue_size ), std::bind( &cloud_callback, this, std::placeholders::_1 ) );
```

## Publish on a topic (PointCloud2)

```C++
// ROS1, from within node
auto nh = getNodeHandle();
ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>( "/cloud_topic", queue_size );

// ROS2, from within node
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> cloud_pub = \
    this->create_publisher<sensor_msgs::msg::PointCloud2>( "/cloud_topics", rclcpp::QoS( queue_size ) );
```

## Message callbacks (Imu, PointCloud2)

```C++
// ROS1, pcl::PointCLoud possible
void imu_callback( const sensor_msgs::ImuConstPtr& imu_msg ) { ... }
void cloud_callback( const pcl::PointCloud<PointT>& cloud_msg ) { ... }

// ROS2, sensor_msgs::msg::PointCloud2
// ::ConstPtr, ::Ptr are deprecated but still possible. New ::ConstSharedPtr, ::SharedPtr
void imu_callback( sensor_msgs::msg::Imu::ConstSharedPtr imu_msg ) { ... }
void cloud_callback( sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg ) { ... }
```

## Parameter handling

```C++
// ROS1, from within node
// look for parameters in global namespace: /my_param
auto nh = getNodeHandle(); 
std::string topic = nh.param<std::string>( "topic_param_name", "default_topic_name" );
// look for parameters in nested namespace /namespace/my_node_name/my_param
auto private_nh = getPrivateNodeHandle(); 
double max_range = private_nh.param<double>( "max_range_param_name", default_value );

// ROS2, from within node, each node has individual set of parameters
// Declare each parameter once!
std::string topic = this->declare_parameter<std::string>( "topic_param_name", "default_topic_name" );
double max_range = this->declare_parameter<double>( "max_range_param_name", default_value );
// If you don't want to declare each parameter once, initialize your node with the following NopeOptions
rclcpp::init(argc, argv);
rclcpp::NodeOptions options;
options.allow_undeclared_parameters(true);
options.automatically_declare_parameters_from_overrides(true);
auto node = std::make_shared<rclcpp::Node>("my_node", options);
// If a parameter is declared more than once, use the following ternary operator
std::string multi_topic = this->has_parameter( "multi_topic_param_name" ) ? 
                              this->get_parameter( "multi_topic_param_name" ).as_string()
                            : this->declare_parameter<std::string>( "multi_topic_param_name", "default_value" );
```
## Manipulating time stamps

```C++
// ROS1, ros::Time has members sec (int32), nsec (int32)
ros::Time stamp = header.stamp;
stamp += ros::Duration( seconds, nanoseconds ):


// ROS2
builtin_interfaces::Time stamp = header.stamp;
// Since arithmetic operators are only declared for rclcpp::Time, we have to instantiate the correct types
stamp = ( rclcpp::Time( stamp ) + rclcpp::Duration( seconds, nanoseconds ) ).operator builtin_interfaces::msg::Time();
// Manipulate the stamp by directly accessing sec and nanosec members
stamp.sec += seconds;
stamp.nanosec += nanoseconds;

```