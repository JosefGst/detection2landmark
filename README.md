# Aprilt tag detections to Landmarks for cartographer Ros 2
![industrial ci badge](https://github.com/JosefGst/detection2landmark/blob/master/.github/workflows/industrial_ci.yaml/badge.svg)

    ros2 run detection2landmark detection2landmark

Or the launch file with all parameters from the config folder
    
    ros2 launch detection2landmark detection2landmark

Detections_topic is the topic name to subscribe from the april tag node.
'detection2landmark.yaml'

    detection2landmark:
        ros__parameters:
            camera_frame: "camF_optical_frame"
            detections_topic: "camF/detections"
            family: "36h11"       # tag family
            rotation_weight: 1e1
            translation_weight: 1e1
