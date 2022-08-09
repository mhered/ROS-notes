# ROS course



## What is ROS?

ROS (Robot Operating System) is the de facto standard for robot programming, is a middleware between your OS and your robot programming, usually made of multiple programs running potentially in different computers. 

4 elements:

1. Plumbing: ROS allows multiple individual programs (aka processes, nodes) running potentially in different computers to communicate and provides device drivers.
2. Basic tools to reuse: simulation, visualization, GUI, data logging
3. Off-the-shelf capabilities (control, planning, perception, mapping manipulation)
4. Ecosystem: software organized in packages, docs, tutorials.

## ROS philosophy

1. Peer to peer: individual programs communicate p2p over a defined API (ROS topics, services)
2. Distributed: comms run over wireless so nodes can be distributed in different machines
3. Multilingual: client libraries for C++, python, matlab, jva etc and you can mix and match
4. Lightweight: existing standalone libraries in thin ROS wrapper 
5. Free and open source

## ROS workspace environment

workspace environment defines context for current workspace

load default workspace with 

```bash
$ source /opt/ros/noetic/setup.bash
```

overlay your catkin workspace with 

```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
```

check current workspace with 

```bash
$ echo $ROS_PACKAGE_PATH
```

see setup

```bash
$ cat ~/.bashrc
```

## ROS master

Manages communication between nodes, every node registers at startup with the master.

start a master with 

```bash
$ roscore
```

## ROS nodes

nodes is the ROS name for the single-purpose programs, individually compiled, executed and managed and organized in packages that make your robot programming 

run a node with 

```bash
$ rosrun package_name node_name
```

 see list of active nodes with 

```bash
$ rosnode list
```

get info on node with 

```bash
$ rosnode info node_name
```

## ROS topics

nodes communicates over topics (aka streams of messages).  Nodes can publish and/or subscribe to topics - typically there is one publisher and n subscribers.

list of active topics with 

```bash
$ rostopic list
```

subscribe to and print the contents of a topic with 

```bash
$ rostopic echo /topic
```

get info about a topic with 

```bash
$ rostopic info /topic
```

## ROS messages

Message structure and format is defined in a text file with .msg extension. Contains arrays of type int, float, string etc and can be nested (one message can contain a message)

see the type of a topic with

```bash
$ rostopic type /topic
```

publish a message to a topic with (TAB for autocomplete)

```bash
$ rostopic pub /topic type args
```

