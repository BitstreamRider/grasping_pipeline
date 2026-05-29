import yasmin
import yasmin_ros
from yasmin_ros.basic_outcomes import ABORT, SUCCEED
from grasping_pipeline.robot_control import GoToNeutral, OpenGripper, GoToWaypoint, GoToAndLookAtPlacementArea, GoBack, GoToNeutralMoveIt
from grasping_pipeline.find_table_planes import FindTablePlanes
from grasping_pipeline.collision_environment import CollisionEnvironment
from grasping_pipeline_msgs.action import ExecuteGrasp, Place
from grasping_pipeline.grasp_method_selector import GraspMethodSelector
from grasping_pipeline_msgs.action import FindGrasppoint
from grasping_pipeline_msgs.srv import FetchImages, CallObjectDetector, CallPoseEstimator, CallDirectGraspPoseEstimator
from grasping_pipeline.check_table_clean import RemoveNonTableObjects

def create_goal_cb(action_type, goal_fields):
    def goal_cb(blackboard):
        goal = action_type.Goal()
        for field in goal_fields:
            if field in blackboard:
                setattr(goal, field, blackboard[field])
        return goal
    return goal_cb


def create_result_handler(result_fields, success_outcome=SUCCEED):
    def handler(blackboard, result):
        for field in result_fields:
            if hasattr(result, field):
                blackboard[field] = getattr(result, field)
        return success_outcome
    return handler

def create_request_handler(service_type, request_fields=None):
    def handler(blackboard):
        req = service_type.Request()
        if request_fields:
            for field in request_fields:
                if field in blackboard:
                    setattr(req, field, blackboard[field])
        return req
    return handler

def get_robot_setup_sm(setup_waypoint, node):
    '''
    Returns a state machine that performs a setup for the robot and brings it into a well defined state.
    '''
    
    sm = yasmin.StateMachine(outcomes=['setup_succeeded', 'setup_failed'], handle_sigint=True)
    #sm.add_state(
    #    "GO_TO_TABLE",
    #    setup_waypoint, 
    #    transitions={
    #        "aborted": "GO_TO_TABLE",
    #        "succeeded" : "GO_TO_NEUTRAL",
    #    },
    #)
    sm.add_state(
        "GO_TO_NEUTRAL",
        GoToNeutral(node), 
        transitions={
            "aborted": "GO_TO_NEUTRAL",
            "succeeded" : "OPEN_GRIPPER",
        },
    )
    sm.add_state(
        "OPEN_GRIPPER",
        OpenGripper(node), 
        transitions={
            "aborted": "GO_TO_NEUTRAL",
            "succeeded" : "setup_succeeded",
        },
    )
    
    return sm

    
def get_execute_grasp_sm(after_grasp_waypoint, node):
    '''
    Returns a state machine that performs all steps necessary to execute a grasp.
    '''
    sm = yasmin.StateMachine(['end_execute_grasp', 'failed_to_grasp'])

    table_waypoint = GoToWaypoint(node, 0.25, 0.41, 0)

    sm.add_state(
        "FIND_TABLE_PLANES",
        FindTablePlanes(node = node),
        transitions={
            "succeeded" : "ADD_COLLISION_OBJECTS",
        },
    )
    sm.add_state(
        "ADD_COLLISION_OBJECTS",
        CollisionEnvironment(node = node),
        transitions={
            "succeeded" : "EXECUTE_GRASP",
        },
    )
    execute_grasp_actionstate = yasmin_ros.ActionState(
        ExecuteGrasp,
        "execute_grasp",
        create_goal_handler=create_goal_cb(ExecuteGrasp, ['grasp_poses', 'grasp_object_name_moveit', 'table_plane_equations']),
        result_handler=create_result_handler(['placement_surface_to_wrist', 'top_grasp']),
        outcomes=[SUCCEED, ABORT]
    )

    sm.add_state(
        "EXECUTE_GRASP",
        execute_grasp_actionstate,
        transitions={
            "succeeded": "RETREAT_AFTER_GRASP",
            'aborted': 'failed_to_grasp',
            'canceled': 'failed_to_grasp',
        }
    )
    sm.add_state(
        "RETREAT_AFTER_GRASP",
        GoBack(node,0.3),
        transitions={
            "succeeded": "GO_TO_NEUTRAL_AFTER_GRASP",
            "aborted": "GO_TO_NEUTRAL_AFTER_GRASP"
        }
    )
    sm.add_state(
        "GO_TO_NEUTRAL_AFTER_GRASP",
        GoToNeutral(node),
        transitions={
            "succeeded": "GO_BACK_TO_TABLE",
            "aborted": "GO_BACK_TO_TABLE"
        }
    )
    sm.add_state(
        "GO_BACK_TO_TABLE",
        after_grasp_waypoint,
        transitions={
            "succeeded": "end_execute_grasp",
            "aborted": "GO_BACK_TO_TABLE"
        }
    )
    return sm

def get_placement_sm(node):
    '''
    Returns a state machine that performs all steps necessary to place an object.
    '''
    sm = yasmin.StateMachine(outcomes=['end_placement', 'failed_to_place'], handle_sigint=True)
    sm.add_state(
        "GO_TO_AND_LOOK_AT_PLACEMENT_AREA",
        GoToAndLookAtPlacementArea(node),
        transitions={
            "aborted": "failed_to_place",
            "succeeded" : "FIND_TABLE_PLANES_PLACEMENT",
        },
    )  
    sm.add_state(
        "FIND_TABLE_PLANES_PLACEMENT",
        FindTablePlanes(node),
        transitions={
            "succeeded" : "PLACEMENT_PLACE",
        }
    )
    sm.add_state(
        "PLACEMENT_PLACE",
         yasmin_ros.ActionState(
            Place, 
            'place_object', 
            create_goal_handler=create_goal_cb(Place, ['placement_area_bb', 'table_plane_equations', 'table_bbs', 'placement_surface_to_wrist']),
        ),
        transitions={
            "succeeded": "end_placement",
            "aborted": "failed_to_place",
        }
    )

    return sm


def get_find_grasp_sm(node):
    '''
    Returns a state machine that performs all steps necessary to get a grasppoint.'''
    find_grasp_sm = yasmin.StateMachine(outcomes=['failed', 'end_find_grasp'], handle_sigint=True)
    image_fetcher_service = yasmin_ros.ServiceState(
        FetchImages,
        'fetch_synchronized_images',
        create_request_handler=create_request_handler(FetchImages),
        response_handler=create_result_handler(['rgb', 'depth'])
    )
    find_grasp_sm.add_state(
        "IMAGE_FETCHER",
        image_fetcher_service,
        transitions={'succeeded': 'CALL_OBJECT_DETECTOR', 'aborted': 'failed'}
    )
    call_object_detector_service = yasmin_ros.ServiceState(
        CallObjectDetector,
        'call_object_detector',
        create_request_handler=create_request_handler(CallObjectDetector, ['rgb', 'depth']),
        response_handler=create_result_handler(['bb_detections', 'mask_detections', 'class_names', 'class_confidences']),
    )
    find_grasp_sm.add_state(
        'CALL_OBJECT_DETECTOR',
        call_object_detector_service,
        transitions={'succeeded': 'GET_TABLE_PLANES', 'aborted': 'failed'}
    )
    find_grasp_sm.add_state(
        'GET_TABLE_PLANES',
        FindTablePlanes(node, enlarge_table_bb_to_floor=True),
        transitions={'succeeded': 'REMOVE_NON_TABLE_OBJECTS'}
    )
    find_grasp_sm.add_state(
        'REMOVE_NON_TABLE_OBJECTS',
        RemoveNonTableObjects(node),
        transitions={'succeeded': 'SELECT_GRASP_METHOD'}
    )  
    find_grasp_sm.add_state(
        "SELECT_GRASP_METHOD",
        GraspMethodSelector(node),
        transitions={'pose_based_grasp': 'CALL_POSE_ESTIMATOR', 'direct_grasp': 'CALL_DIRECT_GRASP_POSE_ESTIMATOR'}
    )
    call_pose_estimator_service = yasmin_ros.ServiceState(
            CallPoseEstimator, 
            '/call_pose_estimator', 
            create_request_handler=create_request_handler(CallPoseEstimator, ['rgb', 'depth', 'bb_detections', 'mask_detections', 'class_names', 'class_confidences']),
            response_handler=create_result_handler(['pose_results', 'class_names', 'class_confidences'])
    )
    find_grasp_sm.add_state(
        'CALL_POSE_ESTIMATOR',
        call_pose_estimator_service,
        transitions={'succeeded': 'FIND_GRASP', 'aborted': 'failed'},
        remappings={'pose_results':'object_poses'} #TODO check if needed
    )
    find_grasp_actionstate = yasmin_ros.ActionState(
            FindGrasppoint, 
            'find_grasppoint', 
            create_goal_handler=create_goal_cb(FindGrasppoint, ['depth', 'class_names', 'object_poses']),
            result_handler=create_result_handler(['grasp_poses', 'grasp_object_bb', 'grasp_object_name']),
        )
    find_grasp_sm.add_state(
        'FIND_GRASP',
        find_grasp_actionstate,
        transitions={'aborted': 'failed', 'succeeded': 'end_find_grasp'}
    )
    direct_grasp_pose_estimator_service = yasmin_ros.ServiceState(
        CallDirectGraspPoseEstimator,
        'call_direct_grasppose_estimator',              
        create_request_handler=create_request_handler(CallDirectGraspPoseEstimator, ['rgb', 'depth', 'bb_detections', 'mask_detections', 'class_names']),
        response_handler=create_result_handler(['grasp_poses', 'grasp_object_bb', 'grasp_object_name'])
    )
    find_grasp_sm.add_state(
        'CALL_DIRECT_GRASP_POSE_ESTIMATOR',
        direct_grasp_pose_estimator_service,
        transitions={'succeeded': 'end_find_grasp', 'aborted': 'failed'},
    )
    
    return find_grasp_sm

def get_object_detector_sm(node):
    '''
    Returns a state machine that performs all steps necessary to detect an object.'''
    sm = yasmin.StateMachine(outcomes=['failed', 'succeeded'])
    image_fetcher_service = yasmin_ros.ServiceState(
        FetchImages,
        'fetch_synchronized_images',
        create_request_handler=create_request_handler(FetchImages),
        response_handler=create_result_handler(['rgb', 'depth'])
    )
    sm.add_state(
        "IMAGE_FETCHER",
        image_fetcher_service,
        transitions={'succeeded': 'CALL_OBJECT_DETECTOR', 'aborted': 'failed'}
    )
    call_object_detector_service = yasmin_ros.ServiceState(
        CallObjectDetector,
        'call_object_detector',  
        create_request_handler=create_request_handler(CallObjectDetector, ['rgb', 'depth']),
        response_handler=create_result_handler(['bb_detections', 'mask_detections', 'class_names', 'class_confidences'])
    )
    sm.add_state(
        'CALL_OBJECT_DETECTOR',
        call_object_detector_service,
        transitions={'succeeded': 'succeeded', 'aborted': 'failed'}
    )
    return sm
    

def get_pose_estimator_sm(node):
    '''
    Returns a state machine that performs all steps necessary to get grasp poses.'''
    sm = yasmin.StateMachine(outcomes=['failed', 'succeeded'])
    
    sm.add_state(
        "SELECT_GRASP_METHOD", 
        GraspMethodSelector(node), 
        transitions={'pose_based_grasp': 'CALL_POSE_ESTIMATOR', 'direct_grasp': 'CALL_DIRECT_GRASP_POSE_ESTIMATOR'}
    )
        
    call_pose_estimator_service = yasmin_ros.ServiceState(
        CallPoseEstimator,
        '/call_pose_estimator',          
        create_request_handler=create_request_handler(CallPoseEstimator, ['rgb', 'depth', 'bb_detections', 'mask_detections', 'class_names', 'class_confidences']),
        response_handler=create_result_handler(['pose_results', 'class_names', 'class_confidences'])
    )
    sm.add_state(
        'CALL_POSE_ESTIMATOR', 
        call_pose_estimator_service,
        transitions={'succeeded': 'FIND_GRASP', 'aborted': 'failed'},
        remappings={'pose_results':'object_poses'})
    
    find_grasp_actionstate = yasmin_ros.ActionState(
        FindGrasppoint,
        'find_grasppoint',  
        create_goal_handler=create_goal_cb(FindGrasppoint, ['depth', 'class_names', 'object_poses']),
        result_handler=['grasp_poses', 'grasp_object_bb', 'grasp_object_name'])

    sm.add_state(
        'FIND_GRASP', 
        find_grasp_actionstate,
        transitions={'aborted': 'failed', 'canceled': 'failed', 'succeeded': 'succeeded'}
    )
    
    direct_grasp_pose_estimator_service = yasmin_ros.ServiceState(
        CallDirectGraspPoseEstimator,
       'call_direct_grasppose_estimator',  
        create_request_handler=create_request_handler(CallDirectGraspPoseEstimator, ['rgb', 'depth', 'bb_detections', 'mask_detections', 'class_names']),
        response_handler=create_result_handler(['grasp_poses', 'grasp_object_bb', 'grasp_object_name']))
    sm.add_state(
        'CALL_DIRECT_GRASP_POSE_ESTIMATOR',
        direct_grasp_pose_estimator_service,
        transitions={'succeeded': 'succeeded', 'aborted': 'failed'},
    )
    return sm
    
    