# One confirmed survivor tag. Published only after confirmation_threshold consistent
# detections, matching the Phase 4 §4.3 requirement.
#
# frame_id in header is 'map': the raycast against /cloud_registered lands the point in
# camera_init/map, and the tag stays anchored to that frame for the rest of the flight.
# stamp is the first_seen time -- the moment the first bbox that led to this track fired.

Header header
uint32 id                # 1-indexed monotonic across the run
int32 grid_x             # floor((position.x - grid_origin_x) / grid_size)
int32 grid_y             # floor((position.y - grid_origin_y) / grid_size)
geometry_msgs/Point position   # world coordinates of the accumulated centroid, metres
float32 confidence       # last bbox confidence (0..1), from the Ultralytics predict call
uint32 n_obs             # detections merged into this track so far
time first_seen          # ROS time when the first constituent detection fired
