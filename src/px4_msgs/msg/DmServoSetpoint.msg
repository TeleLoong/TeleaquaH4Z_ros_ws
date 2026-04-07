# DM Servo setpoint message

uint64 timestamp			# time since system start (microseconds)

float32 position			# Target position (normalized or absolute)
float32 velocity			# Target velocity (optional, can be 0)
bool enable				# Enable/disable servo (true = enable, false = disable)

