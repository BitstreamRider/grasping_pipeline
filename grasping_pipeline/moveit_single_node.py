from grasping_pipeline.moveit_wrapper import MoveitWrapper
class MoveItSingleNode:
    instance = None

    @staticmethod
    def get(tf_wrapper=None, node=None):
        if MoveItSingleNode.instance is None:
            MoveItSingleNode.instance = MoveitWrapper(tf_wrapper, node)
        return MoveItSingleNode.instance