import rospy
import visNull
class TimeHelper:
    """Object containing all time-related functionality.

    This class mainly exists to support both real hardware and (potentially
    faster or slower than realtime) simulation with the same script.
    When running on real hardware, this class uses ROS time functions.
    The simulation equivalent does not depend on ROS.

    Attributes:
        visualizer: No-op object conforming to the Visualizer API used in
            simulation scripts. Maintains the property that scripts should not
            know/care if they are running in simulation or not.
    """
    def __init__(self):
        self.rosRate = None
        self.rateHz = None
        self.visualizer = visNull.VisNull()

    def time(self):
        """Returns the current time in seconds."""
        return rospy.Time.now().to_sec()

    def sleep(self, duration):
        """Sleeps for the provided duration in seconds."""
        rospy.sleep(duration)

    def sleepForRate(self, rateHz):
        """Sleeps so that, if called in a loop, executes at specified rate."""
        if self.rosRate is None or self.rateHz != rateHz:
            self.rosRate = rospy.Rate(rateHz)
            self.rateHz = rateHz
        self.rosRate.sleep()
