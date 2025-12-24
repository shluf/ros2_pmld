# Single object detection result from YOLO

# Bounding box (x, y, width, height) in pixels
int32 x
int32 y
int32 width
int32 height

# Detection metadata
int32 class_id
string class_name
float32 confidence

# Center point (computed from bbox)
float32 center_x
float32 center_y
