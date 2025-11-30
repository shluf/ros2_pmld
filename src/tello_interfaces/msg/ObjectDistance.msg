# Distance measurement between detected object and ArUco marker

# Object detection reference
int32 object_id          # Index in DetectionArray
string object_class      # Class name of detected object
float32 object_center_x  # Object center in pixels
float32 object_center_y

# ArUco marker reference
int32 marker_id          # ArUco marker ID
float32 marker_center_x  # Marker center in pixels
float32 marker_center_y

# Distance measurements
float32 distance_pixels  # Euclidean distance in pixels
float32 distance_meters  # Estimated real-world distance (using marker as scale)

# Confidence
float32 confidence       # Estimation confidence (0-1)
