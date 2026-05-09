from types import SimpleNamespace


def _ns(d):
    return SimpleNamespace(**d)


conf = SimpleNamespace(
    subscribers=[
        _ns({'topic': '/vehicle/steering_cmd', 'type': 'steer_cmd', 'name': 'steering'}),
        _ns({'topic': '/vehicle/throttle_cmd', 'type': 'throttle_cmd', 'name': 'throttle'}),
        _ns({'topic': '/vehicle/brake_cmd', 'type': 'brake_cmd', 'name': 'brake'}),
        _ns({'topic': '/final_waypoints', 'type': 'path_draw', 'name': 'path'}),
    ],
    publishers=[
        _ns({'topic': '/current_pose', 'type': 'pose', 'name': 'current_pose'}),
        _ns({'topic': '/current_velocity', 'type': 'twist', 'name': 'current_velocity'}),
        _ns({'topic': '/vehicle/steering_report', 'type': 'steer', 'name': 'steering_report'}),
        _ns({'topic': '/vehicle/throttle_report', 'type': 'float', 'name': 'throttle_report'}),
        _ns({'topic': '/vehicle/brake_report', 'type': 'float', 'name': 'brake_report'}),
        _ns({'topic': '/vehicle/obstacle', 'type': 'pose', 'name': 'obstacle'}),
        _ns({'topic': '/vehicle/obstacle_points', 'type': 'pcl', 'name': 'obstacle_points'}),
        _ns({'topic': '/vehicle/lidar', 'type': 'pcl', 'name': 'lidar'}),
        _ns({'topic': '/vehicle/traffic_lights', 'type': 'trafficlights', 'name': 'trafficlights'}),
        _ns({'topic': '/vehicle/dbw_enabled', 'type': 'bool', 'name': 'dbw_status'}),
        _ns({'topic': '/image_color', 'type': 'image', 'name': 'image'}),
    ],
)
