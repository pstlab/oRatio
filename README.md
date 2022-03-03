# oRatio

[![Build Travis-CI Status](https://travis-ci.com/pstlab/oRatio.svg?branch=master)](https://travis-ci.org/pstlab/oRatio)

> Dum loquimur, fugerit invida aetas: carpe diem, quam minimum credula postero. (Orazio, Odi, I, 11, 7-8)

oRatio is an Integrated Logic and Constraint based solver which takes inspiration from both Logic Programming (LP) and Constraint Programming (CP).

## Getting started

The oRatio system is invoked with a list of command line arguments representing the locations of the required [compilation units](https://github.com/pstlab/oRatio/wiki/The-RIDDLE-Language) (e.g. domain models, problem instances or plan adaptations) and the desired output file like in the following:

```
oRatio cu_0.rddl cu_1.rddl ... cu_n sol.json
```

As an example, the following code invokes the oRatio solver using the domain model defined in `examples/blocks/blocks_domain.rddl`, the problem instance specified in `examples/blocks/blocks_problem_10.rddl` and, once found a solution, writes it in the `solution.json` file:

```
oRatio examples/blocks/blocks_domain.rddl examples/blocks/blocks_problem_10.rddl solution.json
```

Further information about the RIDDLE language, used for specifying the input files, can be found in the corresponding [wiki](https://github.com/pstlab/oRatio/wiki/The-RIDDLE-Language) page.

## Building oRatio

The basic requirements for building oRatio are:

- [Git](https://git-scm.com/)
- [CMake](https://cmake.org) v3.x
- A C++ compiler

### Building on Linux

The easiest way to install the building requirements on Ubuntu is as follows

```
sudo apt-get install build-essential
sudo apt-get install cmake
```

once the building requirements are installed, move to a desired folder and clone the oRatio repository

```
git clone https://github.com/pstlab/oRatio
```

finally, build oRatio

```
mkdir build
cd build
cmake ..
make
```

### Building on OS X

The easiest way to install the building requirements on OS X consists in downloading and installing the [Command Line Tools](https://developer.apple.com/downloads/) and CMake. Once the building requirements are installed, open a terminal, move to a desired folder and clone the oRatio repository

```
git clone https://github.com/pstlab/oRatio
```

finally, build oRatio

```
mkdir build
cd build
cmake ..
make
```

### Building on Windows

The easiest way to compile oRatio on Windows is through [Visual Studio](https://www.visualstudio.com/). Download and install Visual Studio, download Git and CMake. Start a Visual Studio Command prompt, move to a desired folder and clone the oRatio repository

```
git clone https://github.com/pstlab/oRatio
```

finally, build oRatio

```
mkdir build
cd build
cmake -G "NMake Makefiles" ..
nmake
```

### Building notes

The CMake building script, by default, generates a 'Debug' target resulting in a code that, when executed, performs several checks and is not optimized. A different target can be choosen either from the CMake gui, by setting the `CMAKE_BUILD_TYPE` parameter to the desired target, or from the command line, by providing CMake the same parameter through the `-D` option.

Available targets are:

* **Debug**, emits debug information and do not performs optimization
* **Release**, omits debug information and performs optimization
* **MinSizeRel**, optimizes for smallest binary
* **RelWithDebInfo**, emits debug information and performs optimization

As an example, an optimized target can be generated through the following command

```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

## ROS
Integrating oRatio into [ROS](https://www.ros.org/) is straightforward. Just clone this repository into a [catkin](http://wiki.ros.org/catkin) workspace and compile the workspace. oRatio, in fact, contains a catkin package which, through the implementation of some ROS services and messages, manages the communication to and from other ROS nodes. It is therefore worth describing what these messages and services are in order to fully exploit the potential of oRatio.

### Listening oRatio
By subscribing to the `deliberative_state` topic it is possible to listen the oRatio's state. The ROS message, in particular, has type `deliberative_tier\deliberative_state` and is structured as follows:

```
uint64 reasoner_id
uint8 idle = 0
uint8 reasoning = 1
uint8 executing = 2
uint8 finished = 3
uint8 inconsistent = 4
uint8 deliberative_state
```

Within ROS, in particular, it is possible to create several oRatio reasoners, each identified by an id. Through these messages it is possible to dynamically listen the status of the different reasoners.

### Creating a reasoner
The first step for using oRatio in a ROS environment consists in creating a new reasoner. The creation of a reasoner is done through the `create_reasoner` service. This service has type `deliberative_tier/create_reasoner` which is structured as follows:

```
string[] domain_files
string[] requirements
string[] notify_start
---
uint64 reasoner_id
bool consistent
```

In particular, the service requires a set of paths to domain files, a set of requirements in the riddle language, and a set of predicates, among those defined within the domain files, of which one is interested in being notified of their start. The service returns an id of the reasoner, which is needed to query it later, and a boolean that indicates whether the reasoning problem is consistent or not. Note that the returned consistency is related only to the reading of the reasoning problem. In other words, before the problem is solved, only trivial consistencies/inconsistencies are detected. Creating a new reasoner, however, automatically starts the resolution process. The status of this resolution process can be perceived by subscribing to the `deliberative_state` topic.

### Incremental planning
oRatio allows incremental planning. If during the execution of a plan, in particular, new requirements emerge, these can be communicated to the reasoner, who integrates them into the current plan. It is possible to communicate new requirements through the `create_reasoner` service of type `deliberative_tier/new_requirement`, which is structured as follows:

```
uint64 reasoner_id
string requirement
---
bool consistent
```

Intuitively, the service allows to add a new requirement to the reasoner identified by the `reasoner_id`. Similarly to the reasoner creation service, the incremental planning service is able to detect only trivial consistencies/inconsistencies. The incremental planning, however, triggers the resolution process. The status of this resolution process can be perceived by subscribing to the `deliberative_state` topic.

### Destroying a reasoner
At any time it is possible to destroy a reasoner. The process of destroying the planner is done by means of the `destroy_reasoner` service of type `deliberative_tier/destroy_reasoner`, which is structured as follows:

```
uint64 reasoner_id
---
bool destroyed
```

Intuitively, this service destroys the reasoner identified by the communicated id, returning a bulean indicating whether the destruction process was successful.

### Executing plans
During execution, the executors associated to the reasoners invoke some ROS services that allow the dispatching of the tasks on the invoking layers. In other words, these services constitute the junction point between the reasoning process and the execution process and, therefore, must be implemented according to specific needs.

There are two main services to implement: `can_start`, of type `deliberative_tier/can_start`, which is used to check the current executability of a given task, and `start_task`, of type `deliberative_tier/start_task`, which requires the starting of the execution of a given task. These two services are similar. The `can_start` service, in particular, structured as follows:

```
string task_name
string[] par_names
string[] par_values
---
bool can_start
```

Through this service, specifically, reasoners can ask the invoking modules if a particular task, whose name is `task_name`, with the parameters `par_names` assuming the values `par_values`, can be executed at a specific time. The negative response from the invoking module turns into a delay request at the start of the activity.

The `start_task` service, on the contrary, structured as follows:

```
uint64 reasoner_id
uint64 task_id
string task_name
string[] par_names
string[] par_values
---
bool started
```

Through this service, specifically, reasoners can ask the invoking modules to start the execution of a particular task, whose name is `task_name`, with the parameters `par_names` assuming the values `par_values`. The id of the reasoner and of the task are used by the invoking module to communicate, at a later time, the result of the execution of the task.