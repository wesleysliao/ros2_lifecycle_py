import rclpy

from rclpy.node import Node

from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.msg import TransitionEvent

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetAvailableStates
from lifecycle_msgs.srv import GetAvailableTransitions
from lifecycle_msgs.srv import GetState


class LifecycleNode(Node):

    
    def __init__(self, node_name:str):
        super().__init__(node_name)
        self.state = State.PRIMARY_STATE_UNCONFIGURED

        self.srv_get_state = self.create_service(
                GetState, 
                node_name + '/get_state',
                self.get_state
            )

        self.srv_change_state = self.create_service(
                ChangeState,
                node_name + '/change_state',
                self.change_state
            )

        # self.srv_get_available_states = self.create_service(
        #         , 
        #         node_name + '__get_available_states',
        #         self.get_available_states
        #     )

        # self.srv_get_available_transitions = self.create_service(
        #         ,
        #         node_name + '__get_available_transitions',
        #         self.get_available_transitions
        #     )


        self.pub_transition_event = self.create_publisher(
                TransitionEvent, 
                node_name + '/transition_event',
                1
            )

    def change_state(self, request, response):
        
        if(request.transition.id == Transition.TRANSITION_CREATE):
            response.success = (self.create() == Transition.TRANSITION_CALLBACK_SUCCESS)

        elif(request.transition.id == Transition.TRANSITION_CONFIGURE):
            response.success = (self.configure() == Transition.TRANSITION_CALLBACK_SUCCESS)

        elif(request.transition.id == Transition.TRANSITION_CLEANUP):
            response.success = (self.cleanup() == Transition.TRANSITION_CALLBACK_SUCCESS)

        elif(request.transition.id == Transition.TRANSITION_ACTIVATE):
            response.success = (self.activate() == Transition.TRANSITION_CALLBACK_SUCCESS)

        elif(request.transition.id == Transition.TRANSITION_DEACTIVATE):
            response.success = (self.deactivate() == Transition.TRANSITION_CALLBACK_SUCCESS)

        elif(request.transition.id == Transition.TRANSITION_UNCONFIGURED_SHUTDOWN
              or request.transition.id == Transition.TRANSITION_INACTIVE_SHUTDOWN
              or request.transition.id == Transition.TRANSITION_ACTIVE_SHUTDOWN):
            response.success = (self.shutdown() == Transition.TRANSITION_CALLBACK_SUCCESS)

        elif(request.transition==Transition.TRANSITION_DESTROY):
            response.success = self.destroy()

        else:
            response.success = False

        return response
    
    def get_label(self, msg_type, id):
        for key, value in vars(msg_type).items():
            if(value == id):
                return key
        return None


    def get_state(self, request, response):
        response.current_state = State(id=self.state, label=self.get_label(State, self.state))
        return response 


    def create(self):
        if(self.state == State.PRIMARY_STATE_UNKNOWN):
            self.pub_transition_event.publish(
                TransitionEvent(
                    timestamp = self.get_clock().now().nanoseconds,
                    transition = Transition(
                        id = Transition.TRANSITION_CREATE,
                        label = self.get_label(Transition, Transition.TRANSITION_CREATE)),
                    start_state = State(
                        id = State.PRIMARY_STATE_UNKNOWN,
                        label = self.get_label(State, State.PRIMARY_STATE_UNKNOWN)),
                    goal_state = State(
                        id = State.PRIMARY_STATE_UNCONFIGURED,
                        label = self.get_label(State, State.PRIMARY_STATE_UNCONFIGURED))
                )
            )

            self.state = State.PRIMARY_STATE_UNCONFIGURED
            return Transition.TRANSITION_CALLBACK_SUCCESS 
        else:
            return Transition.TRANSITION_CALLBACK_FAILURE


    def configure(self):
        if(self.state == State.PRIMARY_STATE_UNCONFIGURED):

            self.state = State.TRANSITION_STATE_CONFIGURING

            self.pub_transition_event.publish(
                TransitionEvent(
                    timestamp = self.get_clock().now().nanoseconds,
                    transition = Transition(
                        id = Transition.TRANSITION_CONFIGURE,
                        label = self.get_label(Transition, Transition.TRANSITION_CONFIGURE)),
                    start_state = State(
                        id = State.PRIMARY_STATE_UNCONFIGURED,
                        label = self.get_label(State, State.PRIMARY_STATE_UNCONFIGURED)),
                    goal_state = State(
                        id = self.state,
                        label = self.get_label(State, self.state))
                )
            )

            task_config = self.executor.create_task(self.on_configure)

            self.executor.spin_until_future_complete(task_config)

            result_transition = None
            if(task_config.result() == Transition.TRANSITION_CALLBACK_SUCCESS):
                self.state = State.PRIMARY_STATE_INACTIVE
                result_transition = Transition.TRANSITION_ON_CONFIGURE_SUCCESS

            elif(task_config.result() == Transition.TRANSITION_CALLBACK_FAILURE):
                self.state = State.PRIMARY_STATE_UNCONFIGURED
                result_transition = Transition.TRANSITION_ON_CONFIGURE_FAILURE

            else:
                self.state = State.TRANSITION_STATE_ERRORPROCESSING
                result_transition = Transition.TRANSITION_ON_CONFIGURE_ERROR

            self.pub_transition_event.publish(
                TransitionEvent(
                    timestamp = self.get_clock().now().nanoseconds,
                    transition = Transition(
                        id = result_transition,
                        label = self.get_label(Transition, result_transition)),
                    start_state = State(
                        id = State.PRIMARY_STATE_UNCONFIGURED,
                        label = self.get_label(State, State.PRIMARY_STATE_UNCONFIGURED)),
                    goal_state = State(
                        id = self.state,
                        label = self.get_label(State, self.state))
                )
            )       

            return task_config.result()


    def cleanup(self):
        if(self.state == State.PRIMARY_STATE_INACTIVE):

            self.state = State.TRANSITION_STATE_CLEANINGUP

            self.pub_transition_event.publish(
                TransitionEvent(
                    timestamp = self.get_clock().now().nanoseconds,
                    transition = Transition(
                        id = Transition.TRANSITION_CLEANUP,
                        label = self.get_label(Transition, Transition.TRANSITION_CLEANUP)),
                    start_state = State(
                        id = State.PRIMARY_STATE_INACTIVE,
                        label = self.get_label(State, State.PRIMARY_STATE_INACTIVE)),
                    goal_state = State(
                        id = self.state,
                        label = self.get_label(State, self.state))
                )
            )       

            task_cleanup = self.executor.create_task(self.on_cleanup)

            self.executor.spin_until_future_complete(task_cleanup)

            result_transition = None
            if(task_cleanup.result() == Transition.TRANSITION_CALLBACK_SUCCESS):
                self.state = State.PRIMARY_STATE_UNCONFIGURED
                result_transition = Transition.TRANSITION_ON_CLEANUP_SUCCESS

            else:
                self.state = State.TRANSITION_STATE_ERRORPROCESSING
                result_transition = Transition.TRANSITION_ON_CLEANUP_ERROR
            
            self.pub_transition_event.publish(
                TransitionEvent(
                    timestamp = self.get_clock().now().nanoseconds,
                    transition = Transition(
                        id = result_transition,
                        label = self.get_label(Transition, result_transition)),
                    start_state = State(
                        id = State.PRIMARY_STATE_UNCONFIGURED,
                        label = self.get_label(State, State.PRIMARY_STATE_UNCONFIGURED)),
                    goal_state = State(
                        id = self.state,
                        label = self.get_label(State, self.state))
                )
            )       

            return task_cleanup.result()


    def activate(self):
        if(self.state == State.PRIMARY_STATE_INACTIVE):

            self.state = State.TRANSITION_STATE_ACTIVATING

            self.pub_transition_event.publish(
                TransitionEvent(
                    timestamp = self.get_clock().now().nanoseconds,
                    transition = Transition(
                        id = Transition.TRANSITION_ACTIVATE,
                        label = self.get_label(Transition, Transition.TRANSITION_ACTIVATE)),
                    start_state = State(
                        id = State.PRIMARY_STATE_INACTIVE,
                        label = self.get_label(State, State.PRIMARY_STATE_INACTIVE)),
                    goal_state = State(
                        id = self.state,
                        label = self.get_label(State, self.state))
                )
            )       

            task_activate = self.executor.create_task(self.on_activate)

            self.executor.spin_until_future_complete(task_activate)

            result_transition = None
            if(task_activate.result() == Transition.TRANSITION_CALLBACK_SUCCESS):
                self.state = State.PRIMARY_STATE_ACTIVE
                result_transition = Transition.TRANSITION_ON_ACTIVATE_SUCCESS

            elif(task_activate.result() == Transition.TRANSITION_CALLBACK_FAILURE):
                self.state = State.PRIMARY_STATE_INACTIVE
                result_transition = Transition.TRANSITION_ON_ACTIVATE_FAILURE

            else:
                self.state = State.TRANSITION_STATE_ERRORPROCESSING
                result_transition = Transition.TRANSITION_ON_CONFIGURE_ERROR
            
            self.pub_transition_event.publish(
                TransitionEvent(
                    timestamp = self.get_clock().now().nanoseconds,
                    transition = Transition(
                        id = result_transition,
                        label = self.get_label(Transition, result_transition)),
                    start_state = State(
                        id = State.TRANSITION_STATE_ACTIVATING,
                        label = self.get_label(State, State.TRANSITION_STATE_ACTIVATING)),
                    goal_state = State(
                        id = self.state,
                        label = self.get_label(State, self.state))
                )
            )       

            return task_activate.result()


    def deactivate(self):
        if(self.state == State.PRIMARY_STATE_ACTIVE):

            self.state = State.TRANSITION_STATE_DEACTIVATING

            self.pub_transition_event.publish(
                TransitionEvent(
                    timestamp = self.get_clock().now().nanoseconds,
                    transition = Transition(
                        id = Transition.TRANSITION_DEACTIVATE,
                        label = self.get_label(Transition, Transition.TRANSITION_DEACTIVATE)),
                    start_state = State(
                        id = State.PRIMARY_STATE_ACTIVE,
                        label = self.get_label(State, State.PRIMARY_STATE_ACTIVE)),
                    goal_state = State(
                        id = self.state,
                        label = self.get_label(State, self.state))
                )
            )       

            task_deactivate = self.executor.create_task(self.on_deactivate)

            self.executor.spin_until_future_complete(task_deactivate)

            result_transition = None
            if(task_deactivate.result() == Transition.TRANSITION_CALLBACK_SUCCESS):
                self.state = State.PRIMARY_STATE_INACTIVE
                result_transition = Transition.TRANSITION_ON_DEACTIVATE_SUCCESS

            else:
                self.state = State.TRANSITION_STATE_ERRORPROCESSING
                result_transition = Transition.TRANSITION_ON_DEACTIVATE_ERROR
            
            self.pub_transition_event.publish(
                TransitionEvent(
                    timestamp = self.get_clock().now().nanoseconds,
                    transition = Transition(
                        id = result_transition,
                        label = self.get_label(Transition, result_transition)),
                    start_state = State(
                        id = State.TRANSITION_STATE_DEACTIVATING,
                        label = self.get_label(State, State.TRANSITION_STATE_DEACTIVATING)),
                    goal_state = State(
                        id = self.state,
                        label = self.get_label(State, self.state))
                )
            )       

            return task_deactivate.result()

    def shutdown(self):

        if (self.state == State.PRIMARY_STATE_UNCONFIGURED
            or self.state == State.PRIMARY_STATE_INACTIVE
            or self.state == State.PRIMARY_STATE_ACTIVE):

            if (self.state == State.PRIMARY_STATE_UNCONFIGURED):
                self.pub_transition_event.publish(
                    TransitionEvent(
                        timestamp = self.get_clock().now().nanoseconds,
                        transition = Transition(
                            id = Transition.TRANSITION_UNCONFIGURED_SHUTDOWN,
                            label = self.get_label(Transition, Transition.TRANSITION_UNCONFIGURED_SHUTDOWN)),
                        start_state = State(
                            id = State.PRIMARY_STATE_UNCONFIGURED,
                            label = self.get_label(State, State.PRIMARY_STATE_UNCONFIGURED)),
                        goal_state = State(
                            id = self.state,
                            label = self.get_label(State, self.state))
                    )
                )       

            elif (self.state == State.PRIMARY_STATE_INACTIVE):
                self.pub_transition_event.publish(
                    TransitionEvent(
                        timestamp = self.get_clock().now().nanoseconds,
                        transition = Transition(
                            id = Transition.TRANSITION_INACTIVE_SHUTDOWN,
                            label = self.get_label(Transition, Transition.TRANSITION_INACTIVE_SHUTDOWN)),
                        start_state = State(
                            id = State.PRIMARY_STATE_INACTIVE,
                            label = self.get_label(State, State.PRIMARY_STATE_INACTIVE)),
                        goal_state = State(
                            id = self.state,
                            label = self.get_label(State, self.state))
                    )
                )       

            elif (self.state == State.PRIMARY_STATE_ACTIVE):
                self.pub_transition_event.publish(
                    TransitionEvent(
                        timestamp = self.get_clock().now().nanoseconds,
                        transition = Transition(
                            id = Transition.TRANSITION_ACTIVE_SHUTDOWN,
                            label = self.get_label(Transition, Transition.TRANSITION_ACTIVE_SHUTDOWN)),
                        start_state = State(
                            id = State.PRIMARY_STATE_ACTIVE,
                            label = self.get_label(State, State.PRIMARY_STATE_ACTIVE)),
                        goal_state = State(
                            id = self.state,
                            label = self.get_label(State, self.state))
                    )
                )       
        
            self.state = State.TRANSITION_STATE_SHUTTINGDOWN

            task_shutdown = self.executor.create_task(self.on_shutdown)

            self.executor.spin_until_future_complete(task_shutdown)

            result_transition = None
            if(task_shutdown.result() == Transition.TRANSITION_CALLBACK_SUCCESS):
                self.state = State.PRIMARY_STATE_FINALIZED
                result_transition = Transition.TRANSITION_ON_SHUTDOWN_SUCCESS

            else:
                self.state = State.TRANSITION_STATE_ERRORPROCESSING
                result_transition = Transition.TRANSITION_ON_SHUTDOWN_ERROR
            
            self.pub_transition_event.publish(
                TransitionEvent(
                    timestamp = self.get_clock().now().nanoseconds,
                    transition = Transition(
                        id = result_transition,
                        label = self.get_label(Transition, result_transition)),
                    start_state = State(
                        id = State.TRANSITION_STATE_SHUTTINGDOWN,
                        label = self.get_label(State, State.TRANSITION_STATE_SHUTTINGDOWN)),
                    goal_state = State(
                        id = self.state,
                        label = self.get_label(State, self.state))
                )
            )       

            return task_shutdown.result()


    def destroy(self):
        if(self.state == State.PRIMARY_STATE_FINALIZED):
            self.destroy_node()



    def on_configure(self):
        return Transition.TRANSITION_CALLBACK_SUCCESS

    def on_cleanup(self):
        return Transition.TRANSITION_CALLBACK_SUCCESS

    def on_activate(self):
        return Transition.TRANSITION_CALLBACK_SUCCESS

    def on_deactivate(self):
        return Transition.TRANSITION_CALLBACK_SUCCESS

    def on_error(self):
        return Transition.TRANSITION_CALLBACK_SUCCESS

    def on_shutdown(self):
        return Transition.TRANSITION_CALLBACK_SUCCESS