import yaml
import numpy as np

with open('base_params.yaml', 'r') as f:
    params = yaml.safe_load(f)

# Constants
GRAVITY = 32.174 # ft/s

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


