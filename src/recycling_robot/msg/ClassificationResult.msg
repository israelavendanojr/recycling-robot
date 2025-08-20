# Classification result message
# Contains all information about a single classification inference

# Header with timestamp and frame info
std_msgs/Header header

# Main prediction results
string predicted_class
int32 class_index
float64 confidence

# All class probabilities (parallel arrays)
string[] class_names
float64[] probabilities

# Inference metadata
float64 inference_time_ms
string device_used