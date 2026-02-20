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




import scipy.io
import os
# import magic_formula as mf
import magicformula as mf

model = mf.PacejkaTireModel.from_yaml("./43100_R20_10psi_0IA.yaml")

def load_tire_data(path_fy, path_mz):
    """
    Loads Pacejka coefficients from .mat files (Fy and Mz) and merges them.
    """
    params = {}
    
    # helper to process one file
    def process_file(path):
        if not os.path.exists(path):
            raise FileNotFoundError(f"File not found: {path}")
            
        mat = scipy.io.loadmat(path)
        if 'mfparams' not in mat:
            raise ValueError(f"No 'mfparams' key in {path}")
            
        # mfparams is a structured array. 
        # Structure: mat['mfparams'][0,0]['FIELD_NAME'][0,0] -> value
        # Or mat['mfparams'][0,0] is the void scalar if it's 1x1
        
        struct = mat['mfparams']
        # Check dimensions
        if struct.ndim == 2:
            data = struct[0,0]
        else:
             data = struct
             
        # Iterate over field names
        for name in data.dtype.names:
            value = data[name]
            # Unpack numpy arrays to scalars if possible
            if value.size == 1:
                params[name] = value.item()
            else:
                params[name] = value
                
    print(f"Loading Fy data from {path_fy}...")
    process_file(path_fy)
    
    print(f"Loading Mz data from {path_mz}...")
    process_file(path_mz)
    
    return params

# Initialize Tire Data
tire_mf_params = None
try:
    # Look for tire_data section we just added
    tire_conf = params.get('tire_data', {})
    file_fy = tire_conf.get('mf_file_fy')
    file_mz = tire_conf.get('mf_file_mz')
    
    if file_fy and file_mz:
        # Resolve paths if relative
        if not os.path.isabs(file_fy):
             file_fy = os.path.abspath(file_fy)
        if not os.path.isabs(file_mz):
             file_mz = os.path.abspath(file_mz)
             
        tire_mf_params = load_tire_data(file_fy, file_mz)
        print("Successfully loaded tire data from .mat files.")
    else:
        print("No tire files specified in params. Using defaults.")       
except Exception as e:
    print(f"Warning: Failed to load tire data. Using defaults. Error: {e}")
    tire_mf_params = None
    
if tire_mf_params is None:
    print("Using default Generic Racing Tire model (Pacejka '96).")

# Tire Force Scaling
force_scale = params.get('tireData', {}).get('forceScale', 1.0)

print("Starting YMD Simulation Loop...")

# From here down doesn't work.

# Data acquisition loop
for z in range(len(slip_ratio_range)):
    sx = slip_ratio_range[z]
    #print(f"Processing Slip Ratio: {sx}")
    
    for x in range(len(slip_angle_range)):
        # Body slip angle [rad]
        beta = slip_angle_range[x]
        #print(f"Processing Slip Angle: {beta}")
        
        # Longitudinal and lateral speeds
        # Note: Velocity is in mph from params, convert to ft/s for physics
        velocity_fts = velocity * 1.46667
        Vx = velocity_fts * np.cos(beta)
        Vy = velocity_fts * np.sin(beta)
        
        for y in range(len(front_left_steer_deltas)):
            # Steering angles split on each front tire [rad]
            delta_lf = front_left_steer_deltas[y]
            delta_rf = front_right_steer_deltas[y]
            #print(f"Processing Steering Angle: {delta_lf}, {delta_rf}")
            
            # Yaw rate (initial guess as 0) [rad/s]
            r_guess = 0.0
            r = None
            
            # Lateral acceleration (initial guess as 0) [G]
            Ay_guess = 0.0
            Ay = None
            
            # Loop to find steady-state lateral acceleration
            # Convergence tolerance: 0.001 G
            max_iter = 100
            iter_count = 0
            
            while (Ay is None) or (abs(Ay_guess - Ay) > 1e-3):
                iter_count += 1
                if iter_count > max_iter:
                    print(f"Warning: Convergence failed at SX={sx}, Beta={beta}, Delta={delta_lf}")
                    break
                    
                if r is not None:
                    r_guess = r
                    Ay_guess = Ay
                
                # Slip angles on each tire [rad]
                # alpha = arctan((Vy + r*dist_x) / (Vx +/- r*track/2)) - delta
                # Small angle approximation often used, but full arctan is safer
                # MATLAB code uses linear approximation: (Vy+...)/(Vx+...) - delta
                
                # Front Left
                denom_lf = Vx - r_guess * front_track_width / 2
                #if abs(denom_lf) < 0.1: denom_lf = 0.1 # Avoid div/0
                alpha_lf = (Vy + r_guess * front_to_CG) / denom_lf - delta_lf + front_toe
                
                # Front Right
                denom_rf = Vx + r_guess * front_track_width / 2
                #if abs(denom_rf) < 0.1: denom_rf = 0.1
                alpha_rf = (Vy + r_guess * front_to_CG) / denom_rf - delta_rf - front_toe
                
                # Rear Left
                denom_lr = Vx - r_guess * rear_track_width / 2
                #if abs(denom_lr) < 0.1: denom_lr = 0.1
                alpha_lr = (Vy - r_guess * rear_to_CG) / denom_lr + rear_toe
                
                # Rear Right
                denom_rr = Vx + r_guess * rear_track_width / 2
                #if abs(denom_rr) < 0.1: denom_rr = 0.1
                alpha_rr = (Vy - r_guess * rear_to_CG) / denom_rr - rear_toe
                
                # Load transfers on front/rear axles [lb]
                # Check for max lateral acceleration limit
                '''
                if abs(Ay_guess) > max_lat_accel:
                    dFz_f = front_lat_load_transfer_sens * np.sign(Ay_guess) * max_lat_accel
                    dFz_r = rear_lat_load_transfer_sens * np.sign(Ay_guess) * max_lat_accel
                else:
                '''
                dFz_f = front_lat_load_transfer_sens * Ay_guess
                dFz_r = rear_lat_load_transfer_sens * Ay_guess
                    
                # Normal loads on each tire [N]
                # Convert lbs to Newtons for Magic Formula (1 lb = 4.44822 N)
                # Static loads are calculated in lbs in previous section
                FZ_lf_lbs = front_static_tire_load + dFz_f
                FZ_rf_lbs = front_static_tire_load - dFz_f
                FZ_lr_lbs = rear_static_tire_load + dFz_r
                FZ_rr_lbs = rear_static_tire_load - dFz_r
                
                FZ_lf_N = FZ_lf_lbs * 4.44822
                FZ_rf_N = FZ_rf_lbs * 4.44822
                FZ_lr_N = FZ_lr_lbs * 4.44822
                FZ_rr_N = FZ_rr_lbs * 4.44822
                
                # Tire Pressure (assuming param has it in psi -> Pa, scaling needed?)
                # Code assumes param.IP.pa is available. Let's use generic 12psi -> ~82kPa if not present
                p_pa = 82737.1
                
                # Camber (IA) in radians
                # From params (static_IA is in deg)
                gamma_f = np.deg2rad(params['frontSuspension']['geom']['static_IA']['value'])
                gamma_r = np.deg2rad(params['rearSuspension']['geom']['static_IA']['value'])

                # Call Magic Formula
                # # Returns (Fx, Fy, Mz) in N, N, Nm
                # fx_lf_N, fy_lf_N, mz_lf_Nm = mf.magic_formula(tire_mf_params, sx, alpha_lf, FZ_lf_N, p_pa, gamma_f)
                # fx_rf_N, fy_rf_N, mz_rf_Nm = mf.magic_formula(tire_mf_params, sx, alpha_rf, FZ_rf_N, p_pa, gamma_f)
                # fx_lr_N, fy_lr_N, mz_lr_Nm = mf.magic_formula(tire_mf_params, sx, alpha_lr, FZ_lr_N, p_pa, gamma_r)
                # fx_rr_N, fy_rr_N, mz_rr_Nm = mf.magic_formula(tire_mf_params, sx, alpha_rr, FZ_rr_N, p_pa, gamma_r)

                fx_lf_N, fy_lf_N = model.compute_forces(FZ_lf_N, sx, alpha_lf)
                fx_rf_N, fy_rf_N = model.compute_forces(FZ_rf_N, sx, alpha_rf)
                fx_lr_N, fy_lr_N = model.compute_forces(FZ_lr_N, sx, alpha_lr)
                fx_rr_N, fy_rr_N = model.compute_forces(FZ_rr_N, sx, alpha_rr)
                mz_lf_Nm = 0
                mz_rf_Nm = 0
                mz_lr_Nm = 0
                mz_rr_Nm = 0
                
                # Scale and convert back to Imperial
                # Force: N -> lbs ( / 4.44822)
                # Moment: Nm -> lb-ft ( * 0.737562 )
                
                FX_lf = force_scale * fx_lf_N / 4.44822 
                FY_lf = force_scale * fy_lf_N / 4.44822 
                MZ_lf = force_scale * mz_lf_Nm * 0.737562 
                
                FX_rf = force_scale * fx_rf_N / 4.44822 
                FY_rf = force_scale * fy_rf_N / 4.44822 
                MZ_rf = force_scale * mz_rf_Nm * 0.737562 
                
                FX_lr = force_scale * fx_lr_N / 4.44822 
                FY_lr = force_scale * fy_lr_N / 4.44822 
                MZ_lr = force_scale * mz_lr_Nm * 0.737562 
                
                FX_rr = force_scale * fx_rr_N / 4.44822 
                FY_rr = force_scale * fy_rr_N / 4.44822 
                MZ_rr = force_scale * mz_rr_Nm * 0.737562 
                
                Ax = (FX_lf * np.cos(delta_lf) + FX_lr + FX_rf * np.cos(delta_rf) + FX_rr) / weight
                
                Ay = (FY_lf * np.cos(delta_lf) + FY_lr + FY_rf * np.cos(delta_rf) + FY_rr) / weight
                
                #if abs(Vx) < 1.0:
                #    r = 0.0
                #else:
                r = (Ay * GRAVITY) / Vx
                
                MZ_total = MZ_lf + MZ_rf + MZ_lr + MZ_rr
            # End While Loop
            
            # Store Data
            AxData[x, y, z] = Ax
            AyData[x, y, z] = Ay
            
            M_front_lat =  (FY_lf * np.cos(delta_lf) + FY_rf * np.cos(delta_rf)) * front_to_CG
            M_front_long =  (FY_rf * np.sin(delta_rf) - FY_lf * np.sin(delta_lf)) * front_track_width / 2
            M_rear_lat = (FY_lr + FY_rr) * rear_to_CG
            
            MData[x, y, z] = M_front_lat + M_front_long + M_rear_lat + MZ_total

print("Simulation Complete.")

import matplotlib.pyplot as plt

print("Plotting YMD (2D)...")

# Visualization
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111)

# Normalization Factor for Yaw Moment
# Mz_norm = Mz / (Weight * Wheelbase)
# Weight is in lbs, Wheelbase in ft -> Mz in lb-ft
# This creates a dimensionless coefficient (or G-equivalent unit for moment)
# norm_factor = weight * wheelbase

# Plot slip angle variation lines (Varying SA, Fixed Delta, Fixed SX) -> Red
for y in range(Delta_len):
    for z in range(SX_len):
        Ay_SALine = AyData[:, y, z]
        M_SALine = MData[:, y, z] 
        ax.plot(Ay_SALine, M_SALine, color='r', linewidth=0.5, alpha=0.5)

# Plot steering angle variation lines (Varying Delta, Fixed SA, Fixed SX) -> Blue
for x in range(SA_len):
    for z in range(SX_len):
        Ay_deltaLine = AyData[x, :, z]
        M_deltaLine = MData[x, :, z] 
        ax.plot(Ay_deltaLine, M_deltaLine, color='b', linewidth=0.5, alpha=0.5)

# Plot slip ratio variation lines (Varying SX, Fixed SA, Fixed Delta) -> Black
for x in range(SA_len):
    for y in range(Delta_len):
        Ay_SXLine = AyData[x, y, :]
        M_SXLine = MData[x, y, :] 
        ax.plot(Ay_SXLine, M_SXLine, color='k', linewidth=0.5, alpha=0.5)

# Labels and Grid
ax.set_xlabel('Lateral Acceleration [G]')
ax.set_ylabel('Yaw Moment [lb*ft]')
ax.set_title('Yaw Moment Diagram')
ax.grid(True)
ax.axhline(0, color='black', linewidth=1)
ax.axvline(0, color='black', linewidth=1)

# Legend
from matplotlib.lines import Line2D
custom_lines = [Line2D([0], [0], color='r', lw=2),
                Line2D([0], [0], color='b', lw=2),
                Line2D([0], [0], color='k', lw=2)]
ax.legend(custom_lines, ['Constant Steering', 'Constant Beta', 'Constant Slip Ratio'])
print("Plotting Complete.")
plt.show()


