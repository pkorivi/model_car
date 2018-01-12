# Generic obstacle message
################################################################################


Header header

# ID from tracking
uint32 id

# Odometry of the object (position, orientation, linear and angular velocities along with covariances).
# The pose is defined as the object center prediction. Since we observe objects only partly and predict
# their bounding box, the (bounding box) center (pose) can jump over time. Velocities, however, are
# consistent. See reference_point for comparison.
nav_msgs/Odometry odom

# The object's velocity with covariance without ego motion of the sensor, resp. vehicle. The directions are
# still relative to the frame defined in the header. This field is for convenience, as it can be computed
# with the vehicle's ego-motion and the given odom values.
geometry_msgs/TwistWithCovariance abs_velocity

# The object's anchor pose. While the object center might jump due to changes of the bounding box
# prediction, this point is consistent over time and points to the same location on the tracked object.
geometry_msgs/Point32 reference_point

# The standard deviation of the reference point.
geometry_msgs/Point32 reference_point_sigma

# The object's bounding box as min and max points. They define a cuboid relative to the object's reference
# point and therefore axis-aligned with the object's orientation. [m]
geometry_msgs/Point32 bounding_box_min
geometry_msgs/Point32 bounding_box_max

# The standard deviation of the bounding box size
geometry_msgs/Vector3 bounding_box_sigma

# The contour points of the object
geometry_msgs/Point32[] contour_points

# definition of classes:
uint16 UNCLASSIFIED    = 0
uint16 UNKNOWN_SMALL   = 1
uint16 UNKNOWN_BIG     = 2
uint16 PEDESTRIAN      = 3
uint16 BIKE            = 4
uint16 CAR             = 5
uint16 TRUCK           = 6
uint16 ELEVATED        = 12

# most likely class of this object
uint16 classification

# The higher this value the more reliable the assigned object class [0,1]
float32 classification_certainty

# The first time this object was observed
time first_observed

# The last time this object was observed
time last_observed
