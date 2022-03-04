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
Integrating oRatio into a [ROS](https://www.ros.org/) environment is straightforward! You just need clone this repository into a [catkin](http://wiki.ros.org/catkin) workspace and compile the workspace. That's all!

oRatio contains a catkin package which, through the implementation of some ROS services and messages, manages the communication to and from other ROS nodes. The package provides ROS services for the creation of reasoners, problem solving, their execution and their adaptation during execution. If on the one hand oRatio deals with the management of the reasoners, on the other hand some ROS services must be implemented by the invoking modules, so as to allow the execution of the dispatched activities.

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

Each time a reasoner changes its state, in particular, a `deliberative_tier\deliberative_state` message is communicated to the to the `deliberative_state` topic.

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

In particular, the service requires a set of paths to domain files, a set of requirements in the riddle language, and a set of predicates, among those defined within the domain files, of which one is interested in being notified of their start. The service returns an id of the reasoner, which is needed for querying it later, and a boolean that indicates whether the reasoning problem is consistent or not. Note that the returned consistency is not determined by the resolution process, hence, only trivial consistencies/inconsistencies are detected. Creating a new reasoner, however, automatically triggers the resolution process, whose result can be detected through a subscription to the `deliberative_state` topic.

### Incremental planning
oRatio allows incremental planning. In case new requirements emerge during the execution of a plan, in particular, these can be communicated to a reasoner, which integrates them into the current plan. It is possible to communicate new requirements through the `create_reasoner` service of type `deliberative_tier/new_requirement`, which is structured as follows:

```
uint64 reasoner_id
string requirement
---
bool consistent
```

Intuitively, the service allows to add a new requirement to the reasoner identified by the `reasoner_id`, returning whether the resulting problem is consistent or not. Similarly to the reasoner creation service, the incremental planning service is able to detect only trivial consistencies/inconsistencies. The incremental planning, nonetheless, triggers the resolution process. The result of this resolution process can be detected through a subscription to the `deliberative_state` topic.

### Destroying a reasoner
At any time it is possible to destroy a reasoner. The process of destroying a reasoner is done by means of the `destroy_reasoner` service of type `deliberative_tier/destroy_reasoner`, which is structured as follows:

```
uint64 reasoner_id
---
bool destroyed
```

Intuitively, this service destroys the reasoner identified by the communicated id, returning a boolean indicating whether the destruction process was successful.

### Executing plans
The ultimate goal of a plan is to be executed. For this reason, each reasoner is associated with an executor who allows the execution and adaptation of the plan generated by the resolution process. Updating the state of an executor can be done by means of the `update_executor_state` service of type `deliberative_tier/executor_state_update`, which is structured as follows:

```
uint64 reasoner_id
uint8 pause = 0
uint8 start = 1
uint8 desired_state
---
uint8 new_state
```

Intuitively, communicating `start` as a `desired_state` will start the execution of the plan generated by the reasoner identified by the given reasoner id. Communicating a `pause` as a `desired_state`, on the contrary, will pause the execution of the plan.

### Tasks
During the execution of the plans, the executors need to check the executability of the tasks and to command their effective execution. These tasks, in particular, are defined by means of structures of type `deliberative_tier/task`, structured as follows:

```
uint64 reasoner_id
uint64 task_id
string task_name
string[] par_names
string[] par_values
```

In addition to an identifier of the reasoner which generated it and an identifier of the task itself, each task has a name `task_name`, corresponding to the associated predicate as defined in one of the domain files, a set of parameter names `par_names`, defined in the domain files, and a set of parameter values `par_values`, a value for each parameter, established by the planner.

### Verification of executability and execution
During the execution of the generated plans, the executors need to know if tasks can be started or ended at a certain time and to communicate the start and the end of the task execution. For this reason oRatio expects the invoking module to implement some ROS services. Specifically, the implementation of four ROS services of typr `deliberative_tier/task_service` is required, which are structured as follows:

```
task task
---
bool success
```

The four services are:
1. `can_start`, which returns a boolean indicating whether, at the moment of the service invocation, the task can start;
2. `start_task`, which requires the starting of the given task and returns a boolean indicating whether the task has actually started;
3. `can_end`, which returns a boolean indicating whether, at the moment of the service invocation, the task can end;
4. `end_task`, which requires the ending of the given task and returns a boolean indicating whether the task has actually  ended;

### Closing the tasks
Once the tasks have been started, the reasoners wait to receive a message indicating their termination. Specifically, the termination of a task is done by means of the `task_finished` service of type `deliberative_tier/task_finished`, which is structured as follows:

```
task task
bool success
---
bool ended
```

Note that this service can be invoked by any ROS node, as long as the information identifying the task is available (i.e., the reasoner id and the task id). In addition to the information that identifies the task, the node responsible for managing the task must also communicate the success of the execution. If the task has failed, in particular, it will be the task of the corresponding reasoner to generate an alternative plan, if possible, which allows the achievement of the set objectives.