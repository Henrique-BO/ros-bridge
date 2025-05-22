from carla_ros_bridge.sensor import Sensor
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TwistWithCovarianceStamped

class SpeedSASSensor(Sensor):
    """
    Pseudo-sensor for Speed and Steering Angle
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor for SpeedSASSensor

        :param uid: unique identifier for this object
        :param name: name identifying this object
        :param parent: the parent of this sensor
        :param relative_spawn_pose: the relative spawn pose of this sensor
        :param node: ROS node handle
        :param carla_actor: CARLA actor object (dummy actor)
        :param synchronous_mode: whether the sensor operates in synchronous mode
        """
        super(SpeedSASSensor, self).__init__(uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode)

        # ROS publishers for speed and steering angle
        self.ackermann_publisher = node.new_publisher(AckermannDriveStamped, self.get_topic_prefix(), qos_profile=10)

        # Start listening for updates
        self.listen()

    def destroy(self):
        """
        Destroy the sensor and its publishers
        """
        super(SpeedSASSensor, self).destroy()
        self.node.destroy_publisher(self.ackermann_publisher)

    def sensor_data_updated(self, carla_sensor_data):
        print(f"Sensor data updated: {carla_sensor_data}")
        self.publish_data()

    def publish_data(self):
        """
        Publish speed and steering angle data
        """
        # Access the parent actor (the vehicle)
        parent_actor = self.parent.carla_actor

        if parent_actor is None:
            self.node.logwarn("Parent actor not found for SpeedSASSensor.")
            return

        # Calculate speed from the parent actor's velocity
        velocity = parent_actor.get_velocity()
        speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5

        # Calculate steering angle from the parent actor's control
        control = parent_actor.get_control()
        steering_angle = control.steer * 70.0  # Assuming ±70 degrees max steering angle

        # Publish AckermannDriveStamped data
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header = self.get_msg_header()
        ackermann_msg.drive.speed = speed
        ackermann_msg.drive.steering_angle = steering_angle
        self.ackermann_publisher.publish(ackermann_msg)

        # TODO add noise and/or blackout
