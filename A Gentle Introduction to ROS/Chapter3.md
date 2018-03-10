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
 
