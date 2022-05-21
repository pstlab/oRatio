# ROS

The Robot Operating System ([ROS](https://www.ros.org/)) is a set of software libraries and tools that help you build robot applications. ROS processes are represented as nodes in a graph structure, connected by edges called _topics_. Such nodes can pass _messages_ to one another through _topics_, make _service calls_ to other nodes, provide a _service_ for other nodes, or set or retrieve shared data from a communal database called the parameter server. 

This folder contains a [catkin](http://wiki.ros.org/catkin) package, called _deliberative tier_, providing some services to other ROS nodes for using oRatio into a ROS environment. This means that, in order to use oRatio in a ROS environment, it is sufficient to clone oRatio in a catkin workspace. In order to allow the execution of the planned tasks, on the other hand, the deliberative tier  expects the presence of some ROS services implemented by other nodes, hereinafter called _sequencer tier_.

Details of the passed messages, of the provided services expected of the expected one are provided below.

## Creating reasoners

In order to create a new reasoner, he deliberative tier provides a service called `create_reasoner`. The service type, called `reasoner_creator`, is structured as follows:

```
string[] domain_files
string[] requirements
---
uint64 reasoner_id
bool consistent
```

The service, specifically, is invoked with an array of domain models (`domain_files`) and an array of planning problems (`requirements`), returning, in a non-blocking way, the id (`reasoner_id`) of the just created reasoner and a boolean (`consistent`) which indicates whether any trivial inconsistencies have been detected in the planning problem.

It is worth noting that a call to this service initiates a potentially lengthy process of resolving the communicated planning problem. The result of the reasoning process is hence communicated through a ROS message on a specific topic.

## Detecting the reasoners' states changes

The reasoners' states changes can be detected by subscribing to the `deliberative_state` topic. The state of the reasoners is communicated through a `deliberative_tier/deliberative_state` message which has the following structure:

```
uint64 reasoner_id
uint8 created = 0
uint8 reasoning = 1
uint8 inconsistent = 2
uint8 idle = 3
uint8 executing = 4
uint8 finished = 5
uint8 destroyed = 6
uint8 deliberative_state
```

Intuitively, the message notifies the interested subscribers that the `reasoner_id` planner is currently in the `deliberative_state` state. Creating a new reasoner puts it into a `created` state. The reasoner immediately begins to solve the problem, jumping into the `reasoning` state. In case the planning problem has no solution the reasoner passes into the `inconsistent` state and, from that moment on, it can only be `destroyed`. If, on the other hand, a solution is found, the reasoner goes into the `idle` state, waiting for an execution command by the reactive tier. Upon the arrival of this command, the reasoner passes into the `executing` state and remains there until adaptations are requested, in which case it goes back into into `reasoning`, to manage them and then return, once managed, to the `executing` state (or, if it is not possible manage the required adaptations, into the `inconsistent` state) or until all the scheduled tasks are executed, in which case it jumps into `finished` state.

## Starting the execution

Once a consistent solution has been found, the reasoner puts itself into an idle state, waiting for the invocation from the reactive tier of a service, called `start_execution`, that triggers the execution of the generated plan. The service, whose type is called `deliberative_tier/executor`, has the following structure:

```
uint64 reasoner_id
string[] notify_start
string[] notify_end
---
uint8 new_state
```

The service, specifically, is invoked with the id (`reasoner_id`) of the reasoner whose plan is waiting for execution, and a couple of arrays that indicate the predicates which, before being started (ended), require the approval of the reactive tier.

## Describing tasks

Much of the information exchanged between the deliberative tier and the reactive tier involves tasks. For this reason we have defined a ROS data type, called `deliberative_tier/task`, with the following structure:

```
uint64 reasoner_id
uint64 task_id
string task_name
string[] par_names
string[] par_values
```

Each task, in particular, consists of the id of the reasoner who generated it (`reasoner_id`), an id that identifies the task (`task_id`), the name of the task (`task_name`), and a set of parameter names (`par_names`) with their respective assigned values (`par_values`).

## Checking the executability and executing the tasks

During execution, for those types of tasks for which the notification of the start (end) has been requested by the `start_execution` service, the `can_start` (`can_end`) service, whose type is called `deliberative_tier/task_executor` and having the following structure, is invoked by the deliberative tier to the reactive tier.

```
task task
---
bool success
```

The service, intuitively, asks for the start (end) of a task, returning a boolean (`success`) indicating whether the task can be started (ended). It is worth noting that the permission to start (end) the execution of a task, offered by the reactive tier, does not directly translate into its starting (ending). Suppose, for example, that by the modeled domain two tasks must start at the same time but, during the execution, only one of them is considered executable by the reactive tier, the latter could return false to just one of them, yet both tasks, compatibly with the other involved constraints, should be delayed. Before communicating the start (end) of a task, in particular, the executor requests permission to the reactive tier, delaying, with the propagation of the involved constraints, the start (end) of those tasks for which permission is not granted. Only those tasks which should have started (ended) and which have not been delayed can then be executed (terminated). The beginning (ending) of the execution of a task is communicated to the reactive tier, on a different channel, through the `start_task` (`end_task`) service having the same type.

## Closing tasks

The notification of the termination of an activity can be used in those cases where the termination does not create problems (for example, to turn off a camera and thus save energy). In all other cases, the deliberative tier expects the reactive tier to communicate the termination of the task. This can be done through the `close_task` service, whose type is called `deliberative_tier/task_closer`, having the following structure:

```
task task
bool success
---
bool ended
```

The service, intuitively, invoked by the reactive tier, communicates the end of the `task` task. The `success` field is used to communicate if the task execution was successful. If not, in particular, the corresponding token is removed from the current plan and the plan adaptation procedure, to ensure the maintenance of the causal constraints, is invoked.

## Dynamically adding new requirements

As we have seen in the previous sections, the system offers, during execution, the possibility of dynamically and incrementally adding new requirements to the planning problem. This service is called `new_requirement`. It has a type called `deliberative_tier/requirement_creator` with the following structure:

```
uint64 reasoner_id
string[] requirements
---
bool consistent
```

The service, intuitively, asks for the addition of some `requirements` to the reasoner `reasoner_id`, returning, as in the case of the creation of a new reasoner, a boolean (`consistent`) which indicates whether any trivial inconsistencies have been detected in the planning problem. The service, in a non-blocking way, initiates the potentially lengthy process of resolving the planning problem, communicating the result through the `deliberative_state` message.

## Destroying reasoners

The last ROS service considered concerns the destruction of a reasoner. The service, called `destroy_reasoner`, has a type called `deliberative_tier/reasoner_destroyer` which has the following structure:

```
uint64 reasoner_id
---
bool destroyed
```

The service, intuitively, destroys the no longer needed reasoner `reasoner_id`, releasing all the resources assigned to it, returning a boolean (`destroyed`) indicating whether the procedure was successful.