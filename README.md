# five_axis_tracking
Takes a point in 3d space from a subscribed topic and transforms it to the A & C axes' servo links. Calculates the desired servo angles and publishes it to the respective joint controllers.

## Subscribes to the following topic:
- ball_target_point

## Publishes to the following topics:
- five_axis_robot/joint1_position_controller/command
- five_axis_robot/joint2_position_controller/command
