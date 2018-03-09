# A Gentle Introduction to ROS
Here are some notes  related to ROS.

## Chapter2	Getting started

### 2.3 A minimal example using turtlesim
In three separate terminals, execute these three commands
```
roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
```
If you give your third terminal (the one executing the turtle_teleop_key command) the input focus and press the Up, Down, Left, or Right keys, the turtle will move in response to your
commands, leaving a trail behind it.

### 2.4 Packages
All ROS software is organized into packages. A ROS package is a coherent collection of files,
generally including both executables and supporting files, that serves a specific purpose.
In the example, we used two executables called turtlesim_node and turtle_teleop_key,
both of which are members of the turtlesim package.

**Listing and locating packages**
```
rospack list
```
Each package is defined by a manifest, which is a file called package.xml. This file defines some details about the package, including its name, version, maintainer, and dependencies.The directory containing package.xml is called the package directory. 

To find the directory of a single package, use the rospack find command:
```
rospack find package-name
```
rospack supports tab completion for package names.

**Inspecting a package**

To view the files in a package directory, use a command like this:
```
rosls package-name
```
If you’d like to “go to” a package directory, you can change the current directory to a particular
package, using a command like this:
```
roscd package-name
```
### 2.5 The master (节点管理器, in chinese)
One of the basic goals of ROS is to enable roboticists to design software as a collection of small, mostly independent programs called **nodes that all run at the same time**. For this to work, **those nodes must be able to communicate with one another**. The part of ROS that facilitates this communication is called the **ROS master**. To start the master, use this command:
```
roscore
```

You should **allow the master to continue running for the entire time that you’re using ROS**. One reasonable workflow is to start roscore in one terminal, then open other terminals for your “real” work. There are not many reasons to stop roscore, except when you’ve finished working with ROS. When you reach that point, you can stop the master by typing Ctrl-C in its terminal.

The roscore command shown here is used to **explicitly** start the ROS master. 

### 2.6 Nodes(节点, in chinese)

Once you’ve started roscore, you can run programs that use ROS. A **running instance** of a ROS program is called a **node**.

**Starting nodes**  The basic command to create a node (also known as “running a ROS program”) is rosrun:
```
rosrun  package-name executable-name
```
There are two required parameters to rosrun. The first parameter is a package name. We discussed package names in Section 2.4. The second parameter is simply the name of an executable file within that package.

**Listing nodes**  ROS provides a few ways to get information about the nodes that are running at any particular time. To get a list of running nodes, try this command:
```
rosnode list
```

**Inspecting a node**  You can get some information about a particular node using this command:
```
rosnode info node-name
```
**Killing a node** To kill a node you can use this command:
```
rosnode kill node-name
```
You can also kill a node using the usual Ctrl-C technique. However, that method may not give the node a chance to unregister itself from the master. A symptom of this problem is that the killed node may still be listed by rosnode list for a while
```
rosnode cleanup
```
### 2.7  Topics and messages
The primary mechanism that ROS nodes use to communicate is to send **messages**. Messages in ROS are organized into named **topics**.The idea is that a node that wants to share information will publish messages on the appropriate topic or topics; a node that wants to receive information will **subscribe** to the topic or topics that it’s interested in. The ROS master takes care of ensuring that publishers and subscribers can find each other;**the messages themselves are sent directly from publisher to subscriber**.

**Viewing the graph**  This idea is probably easiest to see graphically, and the easiest way to visualize the publishsubscribe
relationships between ROS nodes is to use this command:
```
rqt_graph
```
The r is for ROS, and the qt refers to the Qt GUI toolkit used to implement the program. 
> The rqt_graph interface. In this graph, the ovals represent nodes, and the directed edges represent publisher-subscriber relationships. T
![The rqt_graph interface](https://lh3.googleusercontent.com/g0nmFqirTJWzMrQbOZXbc0igyvALxmmSZXpscWnda_cZ1ZHGwdnER2WFDoBoEh2WjPNRXKRc37s )


>The complete turtlesim graph, including nodes that rqt_graph classifies as debug nodes.
![enter image description here](https://lh3.googleusercontent.com/wGc_g3ugmNfSD0FPkjhsNDh1Pcsjb97DUpcMZHSWr2aiPKvltQt3e8A8QErxrlzoZNjja7Mtae8)

 - Notice that rqt_graph itself appears as a node.
 - All of these nodes publish messages on a topic called /rosout, to which the node named /rosout subscribes. This topic is one mechanism through which nodes can generate textual log messages.


**Listing topics** To get a list of active topics, use this command:
```
rostopic list
```
**Echoing messages** You can see the actual messages that are being published on a single
topic using the rostopic command:
```
rostopic echo topic-name
```
>This command will dump any messages published on the given topic to the terminal. Type	
> ```
>rostopic echo /turtle1/cmd_vel
>```
>   then you can see the picture below.
>  ![enter image description here](https://lh3.googleusercontent.com/pSsti8_g6pd6a0LGzbP8lFQbhfpkObjbN4nc16gIA9DIrqFQk05trXojO4wKrOFblAGcS15dlCw)

**Measuring publication rates** There are also two commands for measuring the speed at
which messages are published and the bandwidth consumed by those messages:
```
rostopic hz topic-name
rostopic bw topic-name
```
These commands subscribe to the given topic and output statistics in units of messages per second and bytes per second, respectively.

**Inspecting a topic** You can learn more about a topic using the rostopic info command:
```
rostopic info topic-name
```
For example, from this command:
```
rostopic info /turtle1/color_sensor
```
you should see output similar to this:
>Type: turtlesim/Color
>Publishers:
>　　turtlesim (http://donatello:46397/)
>Subscribers: None

The most important part of this output is the very first line, which shows the message type of that topic. In the case of /turtle1/color_sensor, the message type is turtlesim/Color. The word “type” in this context is referring to the concept of a data type.

**Inspecting a message type** To see details about a message type, use a command like  this:
```
rosmsg show message-type-name
```
Let’s try using it on the message type for /turtle1/color_sensor that we found above:
```
rosmsg show turtlesim/Color
```
The output is:
>uint8 r
>uint8 g
>uint8 b

The format is a list of fields, one per line. Each field is defined by a built-in data type (like int8, bool, or string) and a field name. The output above tells us that a turtlesim/Color is a thing that contains three unsigned 8-bit integers called r, g, and b. 

The format is a list of fields, one per line. Each field is defined by a built-in data type (like int8, bool, or string) and a field name.

Another example, one we’ll revisit several times, is geometry_msgs/Twist. This is the message type for the /turtle1/cmd_vel topic, and it is slightly more complicated:
>geometry_msgs/Vector3 linear
float64 x
float64 y
float64 z
geometry_msgs/Vector3 angular
float64 x
float64 y
float64 z

In this case, both linear and angular are **composite fields** whose data type is geometry_msgs/Vector3.

Publishing messages from the command line Most of the time, the work of publishing messages is done by specialized programs. To publish messages by hand , use rostopic:
```
rostopic pub -r rate-in-hz topic-name message-type message-content
```
This command repeatedly publishes the given message on the given topic at the given rate.
The final message content parameter should provide values for all of the fields in the message type, in order. Here’s an example:
```
rostopic pub -r 1 /turtle1/cmd_vel geometry_msgs/Twist ’[2, 0, 0]’ ’[0, 0, 0]’
```
An alternative is to give single parameter specifying all of the fields as a single YAML dictionary. This command (which does, in fact, contain newline characters) is equivalent to the one above, but it explicitly shows the mapping from field names to values: 
```
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist 
"linear:
	x: 2.0
	y: 0.0
	z: 0.0
angular:
	x: 0.0
	y: 0.0
	z: 0.0"
```

**Understanding message type names** Like everything else in ROS, every message type belongs to a specific package. Message type names always contain a slash, and the part before the slash is the name of the containing 
```
	package: package-name/type-name
```
For example, the turtlesim/Color message type breaks down this way:
>![enter image description here](https://lh3.googleusercontent.com/alwPwlNyTBL0Hrk1OFghGZ7JJ6O8bEvvYBpmx4GipJTHiMX40ZA-YBpxuODhdm1nD0cIHh-73R8)

### 2.8.1 A larger example
Start roscore if it’s not already active. Then, in four separate terminals, run these four commands:
```
rosrun turtlesim turtlesim_node __name:=A
rosrun turtlesim turtlesim_node __name:=B
rosrun turtlesim turtle_teleop_key __name:=C
rosrun turtlesim turtle_teleop_key __name:=D
```
This should start two instances of the turtlesim simulator—These should appear in two separate windows—and two instances of the turtlesim teleoperation node.
>![enter image description here](https://lh3.googleusercontent.com/07EoxJcb5ru3SoP5IJIOadfVnH1O4WXkud5TKGHWfWhnDzhHxhPJoXJ1GKyznkl9wSsdXAYyD9I)
A slightly more complex ROS graph, with two turtlesim nodes named A and B and two teleoperation nodes named C and D.

 **Communication via topics is many-to-many**
The main idea here is that topics and messages are used for many-to-many communication. Many publishers and many subscribers can share a single topic.


**Nodes are loosely coupled**
Turtlesim nodes specifically—and most well-designed ROS nodes generally—are loosely coupled. None of the nodes explicitly know about any of the others; their only interactions are indirect, at the level of topics and messages. This independence of nodes, along with the decomposition it facilitates of larger tasks into smaller reusable pieces, is one of the key design features of ROS.

 - Software (like turtle_teleop_key) that produces information can publish that information, without worrying about how that information is consumed.
 - Software (like turtlesim_node) that consumes information can subscribe to the topic or topics containing the data it needs, without worrying about how those data are produced.
 
 ### 2.9 Checking for problems
 One final (for now) command line tool, which can be helpful when ROS is not behaving the way you expect, is roswtf, which can be run with no arguments:
```
roswtf
```
This command performs a broad variety of sanity checks, including examinations of your environment variables, installed files, and running nodes.
