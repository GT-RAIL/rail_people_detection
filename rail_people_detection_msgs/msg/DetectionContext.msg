# This message gives us more information about how much to trust the fields
# about the person

# First, the source of the person's position
#   POSE_FROM_LEGS: pose information received from the leg detector
#   POSE_FROM_FACE: pose information received from the face detector
#   POSE_FROM_OBJECT: pose information received by projecting the object detection into the point cloud
uint32 pose_source
uint32 POSE_FROM_LEGS=0
uint32 POSE_FROM_FACE=1
uint32 POSE_FROM_OBJECT=2
