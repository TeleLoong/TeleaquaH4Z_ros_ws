#!/bin/sh
#
# @name TeleAI MCUUV Configuration (Gazebo Harmonic)
#
# @type Custom Hyper Multicopter-UUV
# @class Underwater Robot
#

. ${R}etc/init.d/rc.mc_defaults

# param set-default MAV_1_CONFIG 102

# param set-default BAT1_A_PER_V 37.8798
# param set-default BAT1_CAPACITY 18000
# param set-default BAT1_V_DIV 11
# param set-default BAT1_N_CELLS 4
# param set-default BAT_V_OFFS_CURR 0.33

# param set-default CA_AIRFRAME 102
# param set-default CA_AIRFRAME 15

param set-default CA_ROTOR_COUNT 12
param set-default CA_R_REV 4080

param set-default CA_ROTOR0_PX 0.467
param set-default CA_ROTOR0_PY 0.585
param set-default CA_ROTOR0_PZ -0.059
param set-default CA_ROTOR0_KM  0.05

param set-default CA_ROTOR1_PX -0.467
param set-default CA_ROTOR1_PY -0.585
param set-default CA_ROTOR1_PZ -0.059
param set-default CA_ROTOR1_KM  0.05

param set-default CA_ROTOR2_PX 0.467
param set-default CA_ROTOR2_PY -0.585
param set-default CA_ROTOR2_PZ -0.069
param set-default CA_ROTOR2_KM -0.05

param set-default CA_ROTOR3_PX -0.467
param set-default CA_ROTOR3_PY 0.585
param set-default CA_ROTOR3_PZ -0.059
param set-default CA_ROTOR3_KM -0.05

param set-default CA_ROTOR0_AZ -1
param set-default CA_ROTOR1_AZ -1
param set-default CA_ROTOR2_AZ -1
param set-default CA_ROTOR3_AZ -1
param set-default CA_ROTOR0_CT 6.5
param set-default CA_ROTOR1_CT 6.5
param set-default CA_ROTOR2_CT 6.5
param set-default CA_ROTOR3_CT 6.5
param set-default CA_ROTOR4_CT 6.5
param set-default CA_ROTOR5_CT 6.5

# --- Motor 1 (Rotor 0): Bow Starboard Horizontal ---
# Pos: (0.5, 0.3, 0.2)  Axis: (1, -1, 0)
param set-default CA_ROTOR4_PX 0.5
param set-default CA_ROTOR4_PY 0.3
param set-default CA_ROTOR4_PZ 0.2
param set-default CA_ROTOR4_AX 1.0
param set-default CA_ROTOR4_AY -1.0
param set-default CA_ROTOR4_AZ 0.0
param set-default CA_ROTOR4_CT 6.5
param set-default CA_ROTOR4_KM 0.0

# --- Motor 2 (Rotor 1): Bow Port Horizontal ---
# Pos: (0.5, -0.3, 0.2)  Axis: (1, 1, 0)
param set-default CA_ROTOR5_PX 0.5
param set-default CA_ROTOR5_PY -0.3
param set-default CA_ROTOR5_PZ 0.2
param set-default CA_ROTOR5_AX 1.0
param set-default CA_ROTOR5_AY 1.0
param set-default CA_ROTOR5_AZ 0.0
param set-default CA_ROTOR5_CT 6.5
param set-default CA_ROTOR5_KM 0.0

# --- Motor 3 (Rotor 2): Stern Starboard Horizontal ---
# Pos: (-0.5, 0.3, 0.2)  Axis: (1, 1, 0)
param set-default CA_ROTOR6_PX -0.5
param set-default CA_ROTOR6_PY 0.3
param set-default CA_ROTOR6_PZ 0.2
param set-default CA_ROTOR6_AX 1.0
param set-default CA_ROTOR6_AY 1.0
param set-default CA_ROTOR6_AZ 0.0
param set-default CA_ROTOR6_CT 6.5
param set-default CA_ROTOR6_KM 0.0

# --- Motor 4 (Rotor 3): Stern Port Horizontal ---
# Pos: (-0.5, -0.3, 0.2)  Axis: (1, -1, 0)
param set-default CA_ROTOR7_PX -0.5
param set-default CA_ROTOR7_PY -0.3
param set-default CA_ROTOR7_PZ 0.2
param set-default CA_ROTOR7_AX 1.0
param set-default CA_ROTOR7_AY -1.0
param set-default CA_ROTOR7_AZ 0.0
param set-default CA_ROTOR7_CT 6.5
param set-default CA_ROTOR7_KM 0.0

# --- Motor 5 (Rotor 4): Bow Starboard Vertical ---
# Pos: (0.5, 0.5, 0.0)  Axis: (0, 0, -1)
param set-default CA_ROTOR8_PX 0.5
param set-default CA_ROTOR8_PY 0.5
param set-default CA_ROTOR8_PZ 0.0
param set-default CA_ROTOR8_AX 0.0
param set-default CA_ROTOR8_AY 0.0
param set-default CA_ROTOR8_AZ -1.0
param set-default CA_ROTOR8_CT 6.5
param set-default CA_ROTOR8_KM 0.0

# --- Motor 6 (Rotor 5): Bow Port Vertical ---
# Pos: (0.5, -0.5, 0.0)  Axis: (0, 0, 1)
param set-default CA_ROTOR9_PX 0.5
param set-default CA_ROTOR9_PY -0.5
param set-default CA_ROTOR9_PZ 0.0
param set-default CA_ROTOR9_AX 0.0
param set-default CA_ROTOR9_AY 0.0
param set-default CA_ROTOR9_AZ 1.0
param set-default CA_ROTOR9_CT 6.5
param set-default CA_ROTOR9_KM 0.0

# --- Motor 7 (Rotor 6): Stern Starboard Vertical ---
# Pos: (-0.5, 0.5, 0.0)  Axis: (0, 0, 1)
param set-default CA_ROTOR10_PX -0.5
param set-default CA_ROTOR10_PY 0.5
param set-default CA_ROTOR10_PZ 0.0
param set-default CA_ROTOR10_AX 0.0
param set-default CA_ROTOR10_AY 0.0
param set-default CA_ROTOR10_AZ 1.0
param set-default CA_ROTOR10_CT 6.5
param set-default CA_ROTOR10_KM 0.0

# --- Motor 8 (Rotor 7): Stern Port Vertical ---
# Pos: (-0.5, -0.5, 0.0)  Axis: (0, 0, -1)
param set-default CA_ROTOR11_PX -0.5
param set-default CA_ROTOR11_PY -0.5
param set-default CA_ROTOR11_PZ 0.0
param set-default CA_ROTOR11_AX 0.0
param set-default CA_ROTOR11_AY 0.0
param set-default CA_ROTOR11_AZ -1.0
param set-default CA_ROTOR11_CT 6.5
param set-default CA_ROTOR11_KM 0.0

param set-default PWM_MAIN_FUNC1 105
param set-default PWM_MAIN_FUNC2 106
param set-default PWM_MAIN_FUNC3 107
param set-default PWM_MAIN_FUNC4 108
param set-default PWM_MAIN_FUNC5 109
param set-default PWM_MAIN_FUNC6 110
param set-default PWM_MAIN_FUNC7 111
param set-default PWM_MAIN_FUNC8 112
param set-default PWM_AUX_FUNC1 101
param set-default PWM_AUX_FUNC2 102
param set-default PWM_AUX_FUNC3 103
param set-default PWM_AUX_FUNC4 104
param set-default CA_MC_NUM_R 4
param set-default CA_UUV_NUM_R 8
param set-default PWM_MAIN_MAX1 1900
param set-default PWM_MAIN_MAX2 1900
param set-default PWM_MAIN_MAX3 1900
param set-default PWM_MAIN_MAX4 1900
param set-default PWM_MAIN_MAX5 1900
param set-default PWM_MAIN_MAX6 1900
param set-default PWM_MAIN_MAX7 1900
param set-default PWM_MAIN_MAX8 1900
param set-default PWM_MAIN_MIN1 1100
param set-default PWM_MAIN_MIN2 1100
param set-default PWM_MAIN_MIN3 1100
param set-default PWM_MAIN_MIN4 1100
param set-default PWM_MAIN_MIN5 1100
param set-default PWM_MAIN_MIN6 1100
param set-default PWM_MAIN_MIN7 1100
param set-default PWM_MAIN_MIN8 1100
param set-default PWM_MAIN_DIS1 1500
param set-default PWM_MAIN_DIS2 1500
param set-default PWM_MAIN_DIS3 1500
param set-default PWM_MAIN_DIS4 1500
param set-default PWM_MAIN_DIS5 1500
param set-default PWM_MAIN_DIS6 1500
param set-default PWM_MAIN_DIS7 1500
param set-default PWM_MAIN_DIS8 1500

param set-default PWM_AUX_MAX1 1900
param set-default PWM_AUX_MAX2 1900
param set-default PWM_AUX_MAX3 1900
param set-default PWM_AUX_MAX4 1900

param set-default PWM_AUX_MIN1 1100
param set-default PWM_AUX_MIN2 1100
param set-default PWM_AUX_MIN3 1100
param set-default PWM_AUX_MIN4 1100
