# A Gentle Introduction to ROS
Here are some notes  related to ROS.

##  Chapter 6
ROS provides a mechanism for **starting the master and many nodes all at once**, **using a file called a launch file**. The use of launch files is widespread through many ROS packages.


### 6.1 Using launch files 
The basic idea is to list, in a specific XML format, a group of nodes that should be started at the same time. Below shows a small example launch file that starts a turtlesim simulator, along with the teleoperation node and the subscriber node. 

 - This file is **saved as example.launch** in the main package directory for the agitr package.
 - And  use **catkin_make** to enable this launch file.
```
<launch>
  <node
	pkg="turtlesim"
	type="turtlesim_node"
	name="turtlesim"
	respawn="true"
  />

  <node
	pkg="turtlesim"
	type="turtle_teleop_key"
	name="teleop_key"
	required="true"
	launch-prefix="xterm -e"
  />

  <node
	pkg="agitr"
	type="subpose"
	name="pose_subscriber"
	output="screen"
  />

</launch>
``` 
**Executing launch files**
To execute a launch file, use the roslaunch command:
```
roslaunch package-name launch-file-name
```
Before starting any nodes, *roslaunch* will determine whether *roscore* is already running and, **if not, start it automatically.**
>Be careful not to confuse rosrun , which starts a single node, with roslaunch , which can start many nodes at once.
>
>An important fact about roslaunch —one that can be easy to forget—is that all of the nodes in a launch file are started at roughly the same time. As a result, **you cannot be sure about the order in which the nodes will initialize themselves.**

### 6.2 Creating launch files 
**Where to place launch files**
As with all other ROS files, each launch file should be associated with a particular package. 
The usual naming scheme is to give launch files names ending with .launch . The simplest place to store launch files is directly in the package directory.
> For example, /HOME/ros/src/agitr/xxx.launch
--------------------------------------------------------------------
**Basic ingredient**
The simplest launch files consist of **a root element** containing **several node elements.**

 **Inserting the root element**
Launch files are XML documents, and every XML document must have exactly one root element. For ROS launch files, the root element is defined by a pair of launch tags:
```
	<launch>
		...
	</launch>
```
All of the other elements of each launch file should be enclosed between these tags.

**Launching nodes**
The heart of any launch file is a collection of node elements, each of which names a single node to launch. A node element looks like this:
```
    <node
	    pkg="package-name"
		type="executable-name"
		name="node-name"
	/>
```
>The trailing slash near the end of the node tag is both important and easy to forget.  It indicates that no closing tag is coming, and that the node element is complete.
>You can also also write the closing tag explicitly:
>``<node pkg=". . . " type=". . . " name=". . . "></node>``
>In fact, this explicit closing tag is needed if the node has children, such as *remap* or *param* elements.

A node element has three required attributes:

 - The ***pkg*** and ***type*** attributes identify which program ROS should run to start this node. These are the same as the two command line arguments to ***rosrun*** , specifying the package name and the executable name, respectively.
 - The ***name*** attribute assigns a name to the node. This overrides any name that the node would normally assign to itself in its call to ros::init .
 
 **Finding node log files**
An important difference between roslaunch and running each node individually using rosrun is that, by default, standard output from launched nodes is redirected to a log file, and does not appear on the console.  The name of this log file is: 
```
∼ /.ros/log/run_id/node_name-number-stdout.log
```

** Directing output to the console**
To override this behavior for a single node, use the output attribute in its node element:
```
output="screen"
```
In addition to the output attribute, which affects only a single node, **we can also force roslaunch to display output** from **all of its nodes**, using the --screen command-line option:
```
	roslaunch --screen package-name launch-file-name
```

**Requesting respawning : I die, and I reborn**
For each node , we can **ask roslaunch to restart it when it terminates**, by using a *respawn* attribute:
```
	respawn="true"
```

**Requiring nodes : I die, and all of you die**
An alternative to *respawn* is to declare that a node is required :
```
	required="true"
```
When **a required node terminates**, roslaunch responds by **terminating all of the other active nodes** and exiting itself.

**Launching nodes in their own windows**
One potential drawback to using roslaunch is that all of the nodes share the same terminal. For nodes that do rely on console input, as turtle_teleop_key does, it may be preferable to retain the separate terminals. *roslaunch* provides a clean way to achieve this effect.
```
	launch-prefix="command-prefix"
```
The idea is that roslaunch will **insert the given prefix at the start of the command line** it constructs internally to execute the given node.　In example.launch , we used this attribute for the teleoperation node:
```
	launch-prefix="xterm -e"
```
Because of this attribute, this node element is roughly equivalent to this command:
```
	xterm -e rosrun turtlesim turtle_teleop_key
```
The ***xterm*** command **starts a simple terminal window**. The **-e** argument tells xterm to **execute the remainder of its command line** (in this case, rosrun turtlesim turtle_teleop_key ) inside itself, in lieu of a new interactive shell.

### 6.3 Launching nodes inside a namespace
The usual way to set the default namespace for a node—a process often called pushing down into a namespace—is to use a launch file, and assign the ***ns*** attribute in its node element:
`` ns="namespace" ``
The program below shows an example launch file that uses this attribute to create two independent turtlesim simulators.
```
<launch>
  <node
	pkg="turtlesim"
	type="turtlesim_node"
	name="turtlesim_node"
	ns="sim1"
  />
  
  <node
	pkg="turtlesim"
	type="turtle_teleop_key"
	name="teleop_key"
	required="true"
	launch-prefix="xterm -e"
	ns="sim1"
  />
  
  <node
	pkg="turtlesim"
	type="turtlesim_node"
	name="turtlesim_node"
	ns="sim2"
  />
  
  <node
	pkg="agitr"
	type="pubvel"
	name="velocity_publisher"
	ns="sim2"
  />

</launch>
```
**Run  ``rosnode list`` then get the result:**
```
/rosout
/sim1/teleop_key
/sim1/turtlesim_node
/sim2/turtlesim_node
/sim2/velocity_publisher
```

**Run `` rostopic list`` then get:**
```
/rosout
/rosout_agg
/sim1/turtle1/cmd_vel
/sim1/turtle1/color_sensor
/sim1/turtle1/pose
/sim2/turtle1/cmd_vel
/sim2/turtle1/color_sensor
/sim2/turtle1/pose
```
We **pushed each simulator node into its own namespace.** The resulting changes to the topic names make the two simulators truly independent, **enabling us to publish different velocity commands to each one.**

In this example, the namespaces specified by the *ns* attributes are themselves relative names. That is, we used the names sim1 and sim2 in a context within the launch file in which the default namespace is the global namespace / . As a result, the default namespaces for our two nodes are resolved to /sim1 and /sim2 .

### 6.4 Remapping names
Remappings are based on the idea of substitution: Each remapping provides an original name and a new name. Each time a node uses any of its remappings’ original names, the ROS client library silently replaces it with the new name from that remapping.

#### Creating remappings
There are two ways create remappings when starting a node.

 1. To remap a name when starting a node from the command line, give the original name and the new name, separated by a := , somewhere on the command line.
```
	original-name:=new-name

	rosrun turtlesim turtlesim_node turtle1/pose:=tim
```
 2. To remap names within a launch file, use a remap element:
```
	<remap from="original-name" to="new-name" />

	<node node-attributes >
		<remap from="original-name" to="new-name" />
		...
	</node>


	<node pkg="turtlesim" type="turtlesim_node"
			name="turtlesim" >
		<remap from="turtle1/pose" to="tim" />
	</node>
```
**All names**, including the original and new names in the remapping itself, **are resolved to global names**, before ROS applies any remappings. As a result, **names that appear in remappings are often relative names.**
