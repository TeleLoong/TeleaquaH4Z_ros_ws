uint32 MESSAGE_VERSION = 0

uint64 timestamp		# time since system start (microseconds)

# Buoyancy restoring torque in body frame (FRD), computed as r x F_buoyancy
float32[3] torque

# TOPICS buoyancy_ff_torque

