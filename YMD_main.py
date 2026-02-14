import yaml
import numpy as np
import param_gui
import ackermann_solver as ack_sol

with open('base_params.yaml', 'r') as f:
    params = yaml.safe_load(f)

# Open GUI to edit parameters
params = param_gui.edit_params(params)

# Constants
GRAVITY = 32.174 # ft/s

## Looping Range 
# Base YMD


## Import Base Parameters

weight = params['mass']['dry_mass']['value'] + params['mass']['driver_mass']['value'] + params['mass']['fuel_mass']['value'] # Total Weight in lbs
wheelbase = params['dimensions']['wheelbase']['value'] / 12 # Wheelbase (Len from axle to axle) of the car in feet

front_weight_dist = params['mass']['y_loc'] / 100 # Percentage of weight on the front of the car as a decimal

front_track_width = params['frontSuspension']['geom']['track_width']['value'] / 12 # conv to feet
rear_track_width = params['rearSuspension']['geom']['track_width']['value'] / 12 # conv to feet

cg_height = params['mass']['z_loc']['value'] / 12 # Center of gravity height in feet

front_rollCenter_height = params['frontSuspension']['geom']['static_roll_center']['value'] / 12 # Height of front roll centre in feet
rear_rollCenter_height = params['rearSuspension']['geom']['static_roll_center']['value'] / 12 # Height of rear roll centre in feet

ackermann = params['steering']['ackermann']['value']/100 # Ackermann percentage as decimal

front_toe = np.deg2rad(params['frontSuspension']['geom']['static_toe']['value']) # Front toe in radians
rear_toe = np.deg2rad(params['rearSuspension']['geom']['static_toe']['value']) # Rear toe in radians

tire_spring_rate = params['tire_params']['tire_stiffness']['value'] * 12 # Tire spring rate in lb/ft

front_spring_stiffness = params['frontSuspension']['stiffness']['spring_rate']['value'] * 12 # Front spring stiffness in lb/ft
rear_spring_stiffness = params['rearSuspension']['stiffness']['spring_rate']['value'] * 12 # Rear spring stiffness in lb/ft

front_arb_stiffness = params['frontSuspension']['stiffness']['arb_stiffness']['value'] * 12 # Front antiroll bar stiffness in lb/ft
rear_arb_stiffness = params['rearSuspension']['stiffness']['arb_stiffness']['value'] * 12 # Rear antiroll bar stiffness in lb/ft

CoP = params['aero_params']['CoP'] / 100 # Center of pressure as a percent of how far forward on the car it is in decimal


## Derived parameters

# Mass Parameters
front_weight = weight * front_weight_dist
rear_weight = weight - front_weight

front_unsprung_mass = params['frontSuspension']['mass']['sprung_mass']['value'] # in lbs
rear_unsprung_mass = params['rearSuspension']['mass']['sprung_mass']['value'] # in lbs

total_unsprung_mass = front_unsprung_mass + rear_unsprung_mass
total_sprung_mass = weight - total_unsprung_mass
front_sprung_mass = front_weight - front_unsprung_mass
rear_sprung_mass = rear_weight - rear_unsprung_mass

# Other dimensions
front_to_CG = wheelbase * (1 - front_weight_dist) # distance from the front axle to the center of gravity
rear_to_CG = wheelbase * front_weight_dist

Izz = (weight / GRAVITY) * ((front_to_CG + rear_to_CG)**2 + front_track_width**2) / 12 # Moment of inertia [lb-ft^2]

# CG / Roll Center / Ride Heights
unsprung_cg_height = params['frontSuspension']['mass']['cg_height']['value'] / 12 # CG height of unsprung mass [ft]

sprung_cg_height = (cg_height * weight - unsprung_cg_height * total_unsprung_mass) / total_sprung_mass # CG height of sprung mass [ft]

roll_axis_height = cg_height - (front_rollCenter_height + (rear_rollCenter_height - front_rollCenter_height) * front_to_CG / wheelbase) # CG height above roll axis [ft]

# Assume x-position of sprung mass CG is the same as total CG
sprung_cg_x = front_to_CG # x-position of sprung mass CG [ft]

roll_axis_angle = np.arctan((rear_rollCenter_height - front_rollCenter_height) / wheelbase) # Inclination angle of roll axis [rad]

roll_axis_offset = (sprung_cg_height - (front_rollCenter_height + (rear_rollCenter_height - front_rollCenter_height) * sprung_cg_x / wheelbase)) * np.cos(roll_axis_angle) # Distance between sprung mass CG and roll axis (orthogonal) [ft]

# Assume the heights of front and rear unsprung masses are the same as the total unsprung mass
front_unsprung_cg_height = unsprung_cg_height # CG height of front unsprung mass [ft]
rear_unsprung_cg_height = unsprung_cg_height # CG height of rear unsprung mass [ft]

## Suspension
# Motion ratios
front_spring_MR = params['frontSuspension']['kinematics']['spring_MR'] # Front spring motion ratio [wheel/spring]
rear_spring_MR = params['rearSuspension']['kinematics']['spring_MR'] # Rear spring motion ratio [wheel/spring]
front_arb_MR = params['frontSuspension']['kinematics']['arb_MR'] # Front ARB motion ratio [wheel/spring]
rear_arb_MR = params['rearSuspension']['kinematics']['arb_MR'] # Rear ARB motion ratio [wheel/spring]

## Aero
# Frontal area [ft^2]
frontal_area = params['aero_params']['frontal_area']['value']

# Air density [lb/ft^3]
air_density = params['aero_params']['rho']['value']

# Aerodynamic coefficient (lift/drag)
aero_coeff = params['aero_params']['Cl']

# Velocity [mph]
velocity = params['simulation_params']['YMD']['velocity']['value']

# Total downforce [lb]
total_downforce = aero_coeff * frontal_area * velocity**2 * air_density / 2

# Front and rear downforces [lb]
front_downforce = total_downforce * CoP
rear_downforce = total_downforce * (1 - CoP)


## Miscellaneous
# Static tire normal loads (excluding transfer) [lb]
front_static_tire_load = (front_weight + front_downforce / 2) / 2
rear_static_tire_load = (rear_weight + rear_downforce / 2) / 2

## Roll Rate Calculation
# Front
front_arb_wheel_rate = 2 * front_arb_stiffness * front_arb_MR**2 # Front wheel rate contributed by ARB [lbf/ft]
front_wheel_rate = front_spring_stiffness * front_spring_MR**2 + front_arb_wheel_rate # Front wheel rate [lbf/ft]
front_ride_rate = (front_wheel_rate * tire_spring_rate) / (front_wheel_rate + tire_spring_rate) # Front ride rate [lbf/ft]
front_roll_rate = front_track_width**2 * front_ride_rate / 2 # Front roll rate [lbf*ft/rad]
front_sprung_roll_rate = front_roll_rate - (wheelbase - sprung_cg_x) * total_sprung_mass * roll_axis_offset / wheelbase # Front roll rate of sprung mass [lbf*ft/rad]

# Rear
rear_arb_wheel_rate = 2 * rear_arb_stiffness * rear_arb_MR**2 # Rear wheel rate contributed by ARB [lbf/ft]
rear_wheel_rate = rear_spring_stiffness * rear_spring_MR**2 + rear_arb_wheel_rate # Rear wheel rate [lbf/ft]
rear_ride_rate = (rear_wheel_rate * tire_spring_rate) / (rear_wheel_rate + tire_spring_rate) # Rear ride rate [lbf/ft]
rear_roll_rate = rear_track_width**2 * rear_ride_rate / 2 # Rear roll rate [lbf*ft/rad]
rear_sprung_roll_rate = rear_roll_rate - sprung_cg_x * total_sprung_mass * roll_axis_offset / wheelbase # Rear roll rate of sprung mass [lbf*ft/rad]

total_roll_rate = front_roll_rate + rear_roll_rate # Total roll rate [lbf*ft/rad]
roll_gradient = np.rad2deg(-total_sprung_mass * roll_axis_offset / (total_roll_rate - total_sprung_mass * roll_axis_offset)) # Current roll gradient [deg/g]

front_lat_load_transfer_sens = total_sprung_mass * (roll_axis_offset * front_sprung_roll_rate / (total_roll_rate - total_sprung_mass * roll_axis_offset) + (wheelbase - sprung_cg_x) * front_rollCenter_height / wheelbase) / front_track_width + front_unsprung_mass * front_unsprung_cg_height / front_track_width # Front lateral load transfer per g [lbf/g]

rear_lat_load_transfer_sens = total_sprung_mass * (roll_axis_offset * rear_sprung_roll_rate / (total_roll_rate - total_sprung_mass * roll_axis_offset) + sprung_cg_x * rear_rollCenter_height / wheelbase) / rear_track_width + rear_unsprung_mass * rear_unsprung_cg_height / rear_track_width # Rear lateral load transfer per g [lbf/g]

TLLTD = front_lat_load_transfer_sens / (front_lat_load_transfer_sens + rear_lat_load_transfer_sens) # Total Lateral Load Transfer Distribution

# Find maximum lateral acceleration [G]
max_lat_accel_front = front_static_tire_load / front_lat_load_transfer_sens
max_lat_accel_rear = rear_static_tire_load / rear_lat_load_transfer_sens
max_lat_accel = min(max_lat_accel_front, max_lat_accel_rear) - 1e-6



## Yaw Moment Diagram Loop Initialization

# Read sweep parameters
slip_angle_sweep = params['simulation_params']['YMD']['sweeps']['slip_angle']
min_slip_angle = np.deg2rad(slip_angle_sweep['min'])
max_slip_angle = np.deg2rad(slip_angle_sweep['max'])
params_slip_angle = int(slip_angle_sweep['points'])

slip_ratio_sweep = params['simulation_params']['YMD']['sweeps']['slip_ratio']
min_slip_ratio = slip_ratio_sweep['min']
max_slip_ratio = slip_ratio_sweep['max']
params_slip_ratio = int(slip_ratio_sweep['points'])

steering_angle_sweep = params['simulation_params']['YMD']['sweeps']['steering_angle']
min_steering = np.deg2rad(steering_angle_sweep['min'])
max_steering = np.deg2rad(steering_angle_sweep['max'])
params_steering = int(steering_angle_sweep['points'])

# Generate ranges
# Linspace includes both endpoints by default, consistent with Matlab behavior usually
slip_angle_range = np.linspace(min_slip_angle, max_slip_angle, params_slip_angle) # SA
slip_ratio_range = np.linspace(min_slip_ratio, max_slip_ratio, params_slip_ratio) # SX
steering_angle_range = np.linspace(min_steering, max_steering, params_steering) # Delta

# Spaces to store data
# Dimensions: (SA, Delta, SX) based on Matlab: zeros(length(SA), length(Delta), length(SX))
SA_len = len(slip_angle_range)
Delta_len = len(steering_angle_range)
SX_len = len(slip_ratio_range)

AxData = np.zeros((SA_len, Delta_len, SX_len))
AyData = np.zeros((SA_len, Delta_len, SX_len))
MData = np.zeros((SA_len, Delta_len, SX_len))

# Spaces to store left & right front steering angles
front_left_steer_deltas = np.zeros(Delta_len)
front_right_steer_deltas = np.zeros(Delta_len)

# Calculate Ackermann geometry effect on front steering angles
for i in range(Delta_len):
    curr_angle = steering_angle_range[i] # Rad
    front_left_steer_deltas[i], front_right_steer_deltas[i] = ack_sol.solve_ackermann(ackermann, curr_angle, front_track_width, wheelbase)


