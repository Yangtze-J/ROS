# A Gentle Introduction to ROS
Here are some notes  related to ROS.

## Chapter3

### 3.1 Creating a workspace and a package
**Creating a workspace**  Packages that you create should live together in a directory called a workspace. You can name your workspace whatever you like, and store the directory anywhere in your account that you prefer.
I use $HOME/ros . Create a subdirectory called src inside the workspace directory to store the source codes.

**Creating a package ** The command to create a new ROS package, which should be run from the src directory of your workspace, looks like this:
```
catkin\_create\_pkg package-name
```
 Actually, this package creation command doesn’t do much: It creates a directory to hold the package and creates two configuration files inside that directory. 

 - The first configuration file is called package.xml,  an inventory.
 - The second file, called CMakeLists.txt,  contains a list of build instructions including what executables should be created, what source files to use to build each of them, and where to find the include files and libraries needed for those executables.

### 3.2  Hello , ROS !

**A simple program**
```
// This is a ROS version of the standard "hello,world" program .

// This header defines the standard ROS classes .
#include <ros/ros.h>

int main(int argc ,char **argv)
{
	//Initialize the ros system
    ros::init(argc,argv,"hello_ros");
    
	//Enstablish this program as a node
    ros::NodeHandle nh;

	//Send some output as a long message
    ROS_INFO_STREAM("Hello, ROS");
}
```

 - The **header file ros/ros.h** includes declarations of the standard ROS classes. You’ll want to include it in every ROS program that you write.
 - The **ros::init function** initializes the ROS client library. Call this once at the beginning of your program.The last parameter is a string containing the default name of your node.
 - The **ros::NodeHandle object** is the main mechanism that your program will use to interact with the ROS system. Creating this object registers your program as a node with the ROS master. 
 - The **ROS_INFO_STREAM** line generates an informational message. This message is sent to several different locations, including the console screen. 

**Compiling the Hello program**
How can you compile and run this program? This is handled by ROS’s build system, called catkin. There are four steps.

  **1. Declaring dependencies** 
First, we need to declare the other packages on which ours depends. For C++ programs, this step is needed primarily to ensure that catkin provides the C++ compiler with the appropriate flags to locate the header files and libraries that it needs.

To list dependencies, edit the CMakeLists.txt in your package directory. The default version of this file has this line:
```find_package(catkin REQUIRED)```
Dependencies on other catkin packages can be added in a COMPONENTS section on this line:
```find_package(catkin REQUIRED COMPONENTS package-names)```
For the hello example, we need one dependency on a package called roscpp, which provides the C++ ROS client library. The required find_package line, therefore, is:
```find_package(catkin REQUIRED COMPONENTS roscpp)```

__________________
We should also list dependencies in the package manifest (package.xml), using the build_depend and run_depend elements:
``<build_depend>package-name</build_depend>``
``<run_depend>package-name</run_depend>``
In our example, the hello program needs roscpp both at build time and at run time, so the
manifest should contain:
``<build_depend>roscpp</build_depend>``
``<run_depend>roscpp</run_depend>``

**Problem: 
Error(s) in /home/yang/ros/src/agitr/package.xml: The manifest (with format version 2) must not contain the following tags: run_depend** 

**Solution:  Do not add these two lines in file package.xml .** 
________________
**2.Declaring an executable**
Next, we need to add two lines to CMakeLists.txt declaring the executable we would like to create. The general form is
```
add_executable(executable-name source-files)
target_link_libraries(executable-name ${catkin_LIBRARIES})
```

**3.Building the workspace**
Once your CMakeLists.txt is set up, you can build your workspace—including compiling all of the executables in all of its packages—using this command:
```
	catkin_make
```
>If you see errors from catkin_make that the header ros/ros.h cannot be found, or “undefined reference” errors on ros::init or other ROS functions, the most likely  reason is that your CMakeLists.txt does not correctly declare a dependency on roscpp.

**4.Sourcing setup.bash** 
The final step is to execute a script called setup.bash, which is created by catkin_make inside the devel subdirectory of your workspace:
```
source devel/setup.bash
```
This automatically-generated script sets several environment variables that enable ROS to find your package and its newly-generated executables. 

**Executing the hello program**
When all of those build steps are complete, your new ROS program is ready to execute using rosrun :
```
rosrun agitr hello
```
>**Don’t forget to start roscore first;
>This program is a node, and nodes need a master to run correctly.** 
 
### 3.3 A Publisher Program
We’ll see how to send randomly-generated velocity commands to a turtlesim turtle, causing it to wander aimlessly.
```
#include <ros/ros.h>
#include <geomerty_msgs/Twist.h>
#include <cstdlib>

int main(int argc, char* argv[])
{
    //initialize the ros system and become a node 
    ros::init(argc, argv, "publish velocity");
    ros::NodeHandle nh;

    //create a publisher object
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
			"turtle1/cmd_vel", 1000);

    //seed the random number generator
    srand(time(0));

    //loop at 2hz until the node is shut down
    ros::Rate rate(2);
    while(ros::ok())
    {
	geometry_msgs::Twist msg;
	msg.linear.x = double(rand())/double(RAND_MAX);
	msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;

	//publish the message
	pub.publish(msg);

	//send a message to ros::out
	ROS_INFO_STREAM("Sending random velocity command":
			<< "linear= " << msg.linear.x
			<< "angualr= " << msg.angular.z);
	rate.sleep();

    }
}
```

**Including the message type declaration**
Every ROS topic is associated with a message type. Each message type has a corresponding C++ header file. You’ll need to #include this header for every message type used in your program, with code like this:
``#include <package_name/type_name.h>``

**Creating a publisher object** 
The work of actually publishing the messages is done by an object of class ros::Publisher . Í 7 A line like this creates the object we need: 
``ros::Publisher pub = node_handle.advertise<message_type>( topic_name, queue_size);``
 - The node_handle is an object of class ros::NodeHandle , one that you created near the start of your program. We’re calling the advertise method of that object.
 - The message_type part inside the angle brackets—formally called the template parameter—is the data type for the messages we want to publish. This should be the name of the class defined in the header discussed above.
 - The *topic_name* is a **string** containing the name of the topic on which we want to publish.
 - The last parameter to advertise is an integer representing the size of the **message queue** for this publisher. In most cases, a reasonably large value, say 1000, is suitable.**The integer value given here is the number of messages**—and not, as you might guess, the number of bytes— that the message queue can hold.
 >If you want to publish messages on multiple topics from the same node, you’ll need to create a separate ros::Publisher object for each topic.
 >Creating the publisher is an expensive operation, so it’s a usually **bad idea to create a new ros::Publisher object each time ** you want to publish a message. **Instead, create one publisher for each topic**, **and use that publisher throughout the execution of your program**.
 
** Creating and filling in the message object**
Next, we create the message object itself. We already referred to the message class when we created the ros::Publisher object.
We used *rosmsg show* (Section 2.7.2) to see that the geometry_msgs/Twist message type has two top-level fields ( linear and angular ), each of which contains three sub-fields ( x , y , and z ).
Because turtlesim ignores the other four fields ( msg.linear.y , msg.linear.z , msg.angular.x , and msg.angular.y ), we leave them with their default value, which happens to be zero.

**Publishing the Message**
After all of that preliminary work, it is very simple to actually publish the message, using the publish method of the ros::Publisher object.
``pub.publish(msg)``
This method adds the given msg the publisher’s outgoing message queue, from which it will be sent as soon as possible to any subscribers of the corresponding topic.

**Checking for node shutdown**
``ros::ok()``
It will return true,until the node has some reason to shun down.

 - ``rosnode kill``
 - an interrupt signal **Ctrl + c**
 - ``ros::shutdown()``
 - start another node with the same name

**Controlling the publishing rate**
The last new element of pubvel is its use of a ros::Rate object:
``ros::Rate(2)``
This object controls how rapidly the loop returns.
Near the end of the loop iteration, we call the sleep method of this object:
``rate.sleep()``
Without this kind of control, the program would publish messages as fast as the computer allows, which can overwhelm publish and subscribe queues and waste computation and network resources.You can confirm that this regulation is working correctly, using 
``rostopic hz``

### A Subscriber program
Continuing to use turtlesim as a test platform, we’ll subscribe to the /turtle1/pose topic, on which *turtlesim_node* publishes
```
$ rosmsg show turtlesim/Pose
	float32 x
	float32 y
	float32 theta
	float32 linear_velocity
	float32 angular_velocity
```

**Writing a callback function**
One important difference between publishing and subscribing is that **a subscriber node doesn’t know when messages will arrive.** To deal with this fact, we must place any code that responds to incoming messages inside a **callback function**, which **ROS calls once for each arriving message**.
A subscriber callback function looks like this:
```
	void function_name(const package_name::type_name &msg) 
	{
		...
	}
```
The package_name and type_name are the same as for publishing: They refer to the message class for the topic to which we plan to subscribe. The body of the callback function then has access to all of the fields in the received message,As always, we must **include the appropriate header** that defines this class.
```
// subpose.cpp
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <iomanip>

// A callback function
void poseMessageReceived(const turtlesim::Pose & msg)
{
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed 
			<< "position=( " << msg.x << ", " << msg.y 
			<< " )" << " direction= " << msg.theta);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "subscribe_to_pose");
    ros::NodeHandle nh;
    //create a subscriber object
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, &poseMessageReceived);

    //let ROS take over
    ros::spin(); 
}

```
In the example, our callback accepts **messages of type turtlesim::Pose** , so the header we need is **turtlesim/Pose.h** .
**Creating a subscriber object: **
To subscribe to a topic, we create a ros::Subscriber object:
```
ros::Subscriber sub = node_handle.subscribe(topic_name, queue_size,
											pointer_to_callback_function);
```
**Giving ROS Control**
The final complication is that ROS will only execute our callback function when we give it explicit permission to do so. There are actually two slightly different ways to accomplish this. One version looks like this:
``ros::spinOnce();``
This code asks ROS to execute all of the pending callbacks from all of the node’s subscriptions, and then return control back to us. The other option looks like this:
``ros::spin();``
This alternative to ros::spinOnce() asks ROS to wait for and execute callbacks until the node shuts down. In other words, *ros::spin()* is roughly equivalent to this loop:
```
while(ros::ok())
{
	ros::spinOnce();
}
```

#### Appendix

 1. The contents in CMakeLists.txt
 
 ```
cmake_minimum_required(VERSION 2.8.3)
project(agitr)

#Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

#Find catkin macros and libraries
#if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz) is used, also find other #catkin packages
find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs turtlesim)
catkin_package ( )

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable(hello hello.cpp)
add_executable(pubvel pubvel.cpp)
add_executable(subpose subpose.cpp)

target_link_libraries(hello ${catkin_LIBRARIES})
target_link_libraries(pubvel ${catkin_LIBRARIES})
target_link_libraries(subpose ${catkin_LIBRARIES})
 ```
 
 2. The contents in package.xml
```
<?xml version="1.0"?>
<package format="2">
  <name>agitr</name>
  <version>0.0.0</version>
  <description>The agitr package</description>
  <maintainer email="yang@todo.todo">yang</maintainer>
  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>turtlesim</build_depend>

  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->
  </export>
</package>
```
