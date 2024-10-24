
import time
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    waypoints = [
        {
            "x": 0.8468844890594482,
            "y": -0.0845317393541336,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.5323170593657491,
            "qw": 0.8465450657278687
        },
        {
            "x": 0.963241457939148,
            "y": 0.5066562294960022,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.7499148066360094,
            "qw": 0.6615344154222641
        },
        {
            "x": 0.6496784687042236,
            "y": 0.9676530957221985,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.9714516320878575,
            "qw": 0.2372377004479645
        },
        {
            "x": 0.17888396978378296,
            "y": 1.0539872646331787,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": -0.9463608910839065,
            "qw": 0.3231115346546367
        },
        {
            "x": -0.13098974525928497,
            "y": 0.5612476468086243,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": -0.7088215549887057,
            "qw": 0.7053878388400194
        }
    ]

    goal_poses = []
    for wp in waypoints:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = wp["x"]
        goal_pose.pose.position.y = wp["y"]
        goal_pose.pose.position.z = wp["z"]
        goal_pose.pose.orientation.x = wp["qx"]
        goal_pose.pose.orientation.y = wp["qy"]
        goal_pose.pose.orientation.z = wp["qz"]
        goal_pose.pose.orientation.w = wp["qw"]
        goal_poses.append(goal_pose)

    while rclpy.ok():
        for goal_pose in goal_poses:
            navigator.goThroughPoses([goal_pose])

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    print('Distance remaining: {:.2f} meters'.format(feedback.distance_remaining))

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
