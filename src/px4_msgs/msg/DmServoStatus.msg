# DM Servo status message

uint64 timestamp			# time since system start (microseconds)

float32 position			# Current position
float32 velocity			# Current velocity
float32 torque				# Current torque
uint8 temperature_mos		# MOSFET temperature
uint8 temperature_rotor		# Rotor temperature
uint8 error_state			# Error state (0 = no error)
bool online				# Online status

