string name
uint32 sys_id
float64 timestamp

# states
vrobots_ros2_msg/Vec3 lin_acc
vrobots_ros2_msg/Vec3 lin_vel
vrobots_ros2_msg/Vec3 lin_pos
float32 altitude

vrobots_ros2_msg/Vec3 ang_acc
vrobots_ros2_msg/Vec3 ang_vel
vrobots_ros2_msg/Vec3 euler
vrobots_ros2_msg/Vec3 euler_dot
vrobots_ros2_msg/Vec4 quaternion

# actuators as states
uint32[] pwm
float32[] actuators
vrobots_ros2_msg/Vec3 force
vrobots_ros2_msg/Vec3 torque

# sensors
vrobots_ros2_msg/Vec3 accelerometer
vrobots_ros2_msg/Vec3 gyroscope
vrobots_ros2_msg/Vec3 magnetometer
float32 barometer
float32 temperature
vrobots_ros2_msg/Vec3 gps_pos
vrobots_ros2_msg/Vec3 gps_vel

# physical properties
float32 mass
vrobots_ros2_msg/Vec3 cg
float32[] moment_arms
vrobots_ros2_msg/Vec3 moi_3x1
float32[] moi_3x3
float32[] extra_props

# collisions and events
vrobots_ros2_msg/Collision[] collisions

# onboard cameras
uint8[] image_data_0
uint8[] image_data_1
