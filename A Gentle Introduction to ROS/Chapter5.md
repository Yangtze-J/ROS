# A Gentle Introduction to ROS
Here are some notes  related to ROS.

## Chapter 5	 Graph resource names

### 5.1 Global names
**Nodes, topics, services, and parameters**  are collectively referred to as **graph resources**. Every graph resource is identified by a short string called a **graph resource name**.

Here are some specific graph resource names that we’ve encountered already:
```
	/teleop_turtle
	/turtlesim
	/turtle1/cmd_vel
	/turtle1/pose
	/run_id
	/count_and_log/set_logger_level
```
These names are all examples of a specific class of names called **global names**. They’re called global names because they make sense anywhere they’re used. 
There are several parts to a global name:

 - A leading slash /, which identifies the name as a **global name**.
 - A sequence of zero or more **namespaces**, separated by slashes.The example names above include two explicit namespaces, called turtle1 and count_and_log.Global names that don’t explicitly mention any namespace—including three of the examples above—are said to be in the global namespace.
 - A **base name** that describes the resource itself. The base names in the example above
are teleop_turtle, turtlesim, cmd_vel, pose, run_id, and set_logger_level.

### 5.2 Relative names
A name that use ROS to provide a default namespace is called a **relative graph resource name**, or simply a **relative name**. The characteristic feature of a relative name is that it lacks a leading slash (/). 
```
	teleop_turtle
	turtlesim
	cmd_vel
	turtle1/pose
	run_id
	count_and_log/set_logger_level
```
The key to understanding relative names is to remember that relative names cannot be matched to specific graph resources unless we know the default namespace that ROS is using to resolve them.

**Resolve relative names**
To resolve a relative name to a global name, ROS attaches the name of the current default namespace to the front of the relative name:
> /turtle1 (default namespace) + cmd_vel (relative name) ⇒ /turtle1/cmd_vel

**Setting the default namespace**
The default namespace that ROS uses is the global namespace (/).
The best and most common method for choosing a different default namespace for a node or group of nodes is to use ns attributes in a launch file. (See Section 6.3.)

 - Most ROS programs, including all C++ programs that call ros::init , accept a command line parameter called __ns , which specifies a default namespace for that program.
 ``__ns:=default-namespace``
 - You can also set the default namespace for every ROS program executed within a shell, using an environment variable.
  ``export ROS_NAMESPACE=default-namespace``
This environment variable is used only when no other default namespace is specified by the __ns parameter.

### 5.3 Private names

 - Private names begin with a tilde (∼) character. 
 - Like relative names, private names do not fully specify the namespace in which they live, and instead rely on the ROS client library to resolve the name to a complete global name.
 - Instead of using the current default namespace, **private names use the name of their node as a namespace**.For instance, in a node whose global name is /sim1/pubvel , the private name *∼ max_vel* would be converted to a global name like this:
```
/sim1/pubvel    +    ~max_vel    -->    /sim/pubvel/max_vel
------------         --------           -------------------
 node name         private name            global name
```
The intuition is that each node has its own namespace for things that are related only to that node, and are not interesting to anyone else.
**Private names** are often used for **parameters**— **roslaunch has a specific feature for setting parameters that are accessible by private names**; see page 113—and services that govern the operation of a node.

### 5.4 Anonymous names
ROS provides one more naming mechanism called anonymous names, which are specifically used to name nodes.
The idea is that a node can, during its call to ros::init , request that a unique name be assigned automatically.
To **request an anonymous name**, a **node** should **pass ros::init_options::Anonymous-Name** as a **fourth parameter** to ros::init :
```
ros::init(argc, argv, base_name, ros::init_options::AnonymousName);
```
The effect of this extra option is to append some extra text to the given base name, ensuring that the node’s name is unique.
```
// A program calls its own node name
#include<ros/ros.h>
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "anon", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
	rosRate rate(1);
	while(ros::ok())
	{
	    ROS_INFO_STREAM("This message is from"
					    << ros::this_node::getName());
		rate.sleep();
	}
}

//Results
/anon_1376942789079547655
/anon_1376942789079550387
/anon_1376942789080356882
```
It requests an anonymous name, we are free to run as many simultaneous copies of that program.
