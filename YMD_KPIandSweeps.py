import yaml
import numpy as np
import param_gui_sweep
import ackermann_solver as ack_sol
import scipy.io
import os
import magicformula as mf
import matplotlib.pyplot as plt
import copy
from datetime import datetime

# Helper to modify nested dictionary
def set_nested_val(d, path_str, val):
    if not path_str: return
    keys = path_str.split('.')
    current = d
    for k in keys[:-1]:
        current = current[k]
        
    if isinstance(current[keys[-1]], dict) and 'value' in current[keys[-1]]:
        current[keys[-1]]['value'] = val
    else:
        current[keys[-1]] = val

def load_tire_data(path_fy, path_mz):
    params = {}
    def process_file(path):
        if not os.path.exists(path): return
        mat = scipy.io.loadmat(path)
        if 'mfparams' not in mat: return
        struct = mat['mfparams']
        data = struct[0,0] if struct.ndim == 2 else struct
        for name in data.dtype.names:
            value = data[name]
            params[name] = value.item() if value.size == 1 else value
    process_file(path_fy)
    process_file(path_mz)
    return params

def get_tire_model(params):
    tire_mf_params = None
    try:
        tire_conf = params.get('tire_data', {})
        file_fy = tire_conf.get('mf_file_fy')
        file_mz = tire_conf.get('mf_file_mz')
        if file_fy and file_mz:
            if not os.path.isabs(file_fy): file_fy = os.path.abspath(file_fy)
            if not os.path.isabs(file_mz): file_mz = os.path.abspath(file_mz)
            tire_mf_params = load_tire_data(file_fy, file_mz)
    except:
        pass
    # For now relying on MF existing integration
    model = mf.PacejkaTireModel.from_yaml("./43100_R20_10psi_0IA.yaml")
    return model

def extract_kpis(AyData, MData, SA_range, Delta_range):
    kpis = {}
    # Assume we extract KPIs from the first slip ratio slice z=0
    # 1. Steady State Grip Limit
    max_ay_mz0 = 0.0
    for x in range(len(SA_range)):
        M_line = MData[x, :, 0]
        Ay_line = AyData[x, :, 0]
        for i in range(len(M_line)-1):
            if M_line[i] * M_line[i+1] <= 0:
                dy = Ay_line[i+1] - Ay_line[i]
                dm = M_line[i+1] - M_line[i]
                if dm != 0:
                    ay_cross = Ay_line[i] - dy * M_line[i] / dm
                    if ay_cross > max_ay_mz0: max_ay_mz0 = ay_cross

    for y in range(len(Delta_range)):
        M_line = MData[:, y, 0]
        Ay_line = AyData[:, y, 0]
        for i in range(len(M_line)-1):
            if M_line[i] * M_line[i+1] <= 0:
                dy = Ay_line[i+1] - Ay_line[i]
                dm = M_line[i+1] - M_line[i]
                if dm != 0:
                    ay_cross = Ay_line[i] - dy * M_line[i] / dm
                    if ay_cross > max_ay_mz0: max_ay_mz0 = ay_cross
    kpis['Grip Limit (Acceleration)'] = max_ay_mz0

    # 2. Limit Balance
    Ay_flat = AyData[:, :, 0]
    Mz_flat = MData[:, :, 0]
    idx_max_ay = np.unravel_index(np.argmax(Ay_flat), Ay_flat.shape)
    kpis['Limit Balance (Yaw Moment)'] = Mz_flat[idx_max_ay]

    # 3. Control (Slope of Mz vs Delta at Beta 0)
    idx_beta_0 = np.argmin(np.abs(SA_range))
    Mz_beta_0 = MData[idx_beta_0, :, 0]
    Delta_deg = np.rad2deg(Delta_range)
    # Exclude extremities for slope -> use central points or simple linear fit
    poly_ctrl = np.polyfit(Delta_deg, Mz_beta_0, 1)
    kpis['Control'] = -poly_ctrl[0] # [lb-ft/deg]

    # 4. Stability (Slope of Mz vs Beta at Delta 0)
    idx_delta_0 = np.argmin(np.abs(Delta_range))
    Mz_delta_0 = MData[:, idx_delta_0, 0]
    SA_deg = np.rad2deg(SA_range)
    poly_stab = np.polyfit(SA_deg, Mz_delta_0, 1)
    kpis['Stability'] = poly_stab[0] # [lb-ft/deg]

    return kpis

def generate_ymd(params, velocity_override=None):
    GRAVITY = 32.174
    model = get_tire_model(params)
    force_scale = params.get('tireData', {}).get('forceScale', 1.0)
    
    weight = params['mass']['dry_mass']['value'] + params['mass']['driver_mass']['value'] + params['mass']['fuel_mass']['value'] # Total Weight in lbs
    wheelbase = params['dimensions']['wheelbase']['value'] / 12 # Wheelbase (Len from axle to axle) of the car in feet
    
    front_weight_dist = params['mass']['x_loc'] # / 100 # Percentage of weight on the front of the car as a decimal
    
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
    
    Izz = (weight / GRAVITY) * ((front_to_CG + rear_to_CG)**2 + front_track_width**2) # Moment of inertia [lb-ft^2]
    
    
    # CG / Roll Center / Ride Heights
    unsprung_cg_height = params['frontSuspension']['mass']['cg_height']['value'] / 12 # CG height of unsprung mass [ft]
    #print(f"unsprung CG = {unsprung_cg_height}")
    sprung_cg_height = (cg_height * weight - unsprung_cg_height * total_unsprung_mass) / total_sprung_mass # CG height of sprung mass [ft]
    #print(f"spung cg = {sprung_cg_height}")
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
    if velocity_override is not None:
        velocity = velocity_override
    
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

    slip_angle_range = np.linspace(min_slip_angle, max_slip_angle, params_slip_angle)
    if min_slip_angle < 0.0 and max_slip_angle > 0.0 and not np.any(np.isclose(slip_angle_range, 0.0)):
        slip_angle_range = np.sort(np.append(slip_angle_range, 0.0))
        params_slip_angle += 1
    slip_ratio_range = np.linspace(min_slip_ratio, max_slip_ratio, params_slip_ratio) # SX

    steering_angle_range = np.linspace(min_steering, max_steering, params_steering)
    if min_steering < 0.0 and max_steering > 0.0 and not np.any(np.isclose(steering_angle_range, 0.0)):
        steering_angle_range = np.sort(np.append(steering_angle_range, 0.0))
        params_steering += 1
    
    # Spaces to store data
    # Dimensions: (SA, Delta, SX) 
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
                    
        #print(f"Loading Fy data from {path_fy}...")
        process_file(path_fy)
        
        #print(f"Loading Mz data from {path_mz}...")
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
            #print("Successfully loaded tire data from .mat files.")
        else:
            print("No tire files specified in params. Using defaults.")       
    except Exception as e:
        print(f"Warning: Failed to load tire data. Using defaults. Error: {e}")
        tire_mf_params = None
        
    if tire_mf_params is None:
        print("Using default Generic Racing Tire model (Pacejka '96).")
    
    # Tire Force Scaling
    force_scale = params.get('tireData', {}).get('forceScale', 1.0)
    
    #print("Starting YMD Simulation Loop...")
    
    
    for z in range(len(slip_ratio_range)):
        # Static for now (len 1, val 0)
        sx = slip_ratio_range[z]
        
        
        for x in range(len(slip_angle_range)):
            # Body slip angle [rad]
            beta = slip_angle_range[x]
            
            # Longitudinal and lateral speeds
            # Note: Velocity is in mph from params, converts to ft/s for physics
            velocity_fts = velocity * 1.46667
            Vx = velocity_fts * np.cos(beta)
            Vy = velocity_fts * np.sin(beta)
            
            for y in range(len(front_left_steer_deltas)):
                # Steering angles split on each front tire [rad]
                delta_lf = front_left_steer_deltas[y]
                delta_rf = front_right_steer_deltas[y]
                
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
                
                while (Ay is None) or (abs(float(Ay) - float(Ay_guess)) > 1e-3):
                    iter_count += 1
                    if iter_count > max_iter:
                        print(f"Warning: Convergence failed at SX={sx}, Beta={beta}, Delta={delta_lf}")
                        break
                    # Feed back converged values into the next iteration
                    if r is not None:
                        r_guess = float(r)
                    if Ay is not None:
                        Ay_guess = float(Ay)
    
                    # Slip angles on each tire [rad]
                    eps = 1e-6
                    # Front Left
                    denom_lf = Vx - r_guess * front_track_width / 2
                    denom_lf = np.sign(denom_lf) * max(abs(denom_lf), eps)
                    alpha_lf = float(-np.arctan2((Vy + r_guess * front_to_CG), denom_lf) - delta_lf - front_toe)
    
                    # Front Right
                    denom_rf = Vx + r_guess * front_track_width / 2
                    denom_rf = np.sign(denom_rf) * max(abs(denom_rf), eps)
                    alpha_rf = float(-np.arctan2((Vy + r_guess * front_to_CG), denom_rf) - delta_rf + front_toe)
    
                    # Rear Left (no steering angle)
                    denom_lr = Vx - r_guess * rear_track_width / 2
                    denom_lr = np.sign(denom_lr) * max(abs(denom_lr), eps)
                    alpha_lr = float(-np.arctan2((Vy - r_guess * rear_to_CG), denom_lr) + rear_toe)
    
                    # Rear Right (no steering angle)
                    denom_rr = Vx + r_guess * rear_track_width / 2
                    denom_rr = np.sign(denom_rr) * max(abs(denom_rr), eps)
                    alpha_rr = float(-np.arctan2((Vy - r_guess * rear_to_CG), denom_rr) - rear_toe)
                    
                    if abs(Ay_guess) > max_lat_accel:
                        dFz_f = front_lat_load_transfer_sens * np.sign(Ay_guess) * max_lat_accel
                        dFz_r = rear_lat_load_transfer_sens * np.sign(Ay_guess) * max_lat_accel
                    else:
                        dFz_f = front_lat_load_transfer_sens * Ay_guess
                        dFz_r = rear_lat_load_transfer_sens * Ay_guess
                        
                    # Normal loads on each tire [N]
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
                    p_pa = 82737.1
                    
                    # Camber (IA) in radians
                    # From params (static_IA is in deg)
                    gamma_f = np.deg2rad(params['frontSuspension']['geom']['static_IA']['value'])
                    gamma_r = np.deg2rad(params['rearSuspension']['geom']['static_IA']['value'])
    
                    # Tyre Model
                    # Change with updated models
                    fx_lf_N, fy_lf_N = model.compute_forces(FZ_lf_N, sx, alpha_lf)
                    fx_rf_N, fy_rf_N = model.compute_forces(FZ_rf_N, sx, alpha_rf)
                    fx_lr_N, fy_lr_N = model.compute_forces(FZ_lr_N, sx, alpha_lr)
                    fx_rr_N, fy_rr_N = model.compute_forces(FZ_rr_N, sx, alpha_rr)
                    mz_lf_Nm = 0
                    mz_rf_Nm = 0
                    mz_lr_Nm = 0
                    mz_rr_Nm = 0
                    
                    # Scale and convert back to Imperial
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
                    
                    Ax = float((FX_lf * np.cos(delta_lf) + FX_lr + FX_rf * np.cos(delta_rf) + FX_rr) / weight)
                    Ay = float((FY_lf * np.cos(delta_lf) + FY_lr + FY_rf * np.cos(delta_rf) + FY_rr) / weight)
                    
                    MZ_total = float(MZ_lf + MZ_rf + MZ_lr + MZ_rr)
                    
                    if abs(Vx) < 1.0:
                        r = 0.0
                    else:
                        r = (Ay * GRAVITY) / Vx
                    
                    # Check convergence
                    if abs(Ay - Ay_guess) < 1e-3:
                        Ay_guess = Ay
                        r_guess = (Ay * GRAVITY) / Vx if abs(Vx) > 1.0 else 0.0
                        break
                    
                    # Update guesses for next iteration
                    Ay_guess = Ay
                    r_guess = r
                # End While Loop
                
                # Positive Y = Right, Positive X = Forward
                FY_veh_lf = FX_lf * np.sin(delta_lf) + FY_lf * np.cos(delta_lf)
                FY_veh_rf = FX_rf * np.sin(delta_rf) + FY_rf * np.cos(delta_rf)
                FY_veh_lr = FY_lr
                FY_veh_rr = FY_rr
                
                FX_veh_lf = FX_lf * np.cos(delta_lf) - FY_lf * np.sin(delta_lf)
                FX_veh_rf = FX_rf * np.cos(delta_rf) - FY_rf * np.sin(delta_rf)
                FX_veh_lr = FX_lr
                FX_veh_rr = FX_rr
                
                # Store Data
                AxData[x, y, z] = Ax
                AyData[x, y, z] = Ay
                
                # Calculate Moments around the CG
                # Positive Moment = Turning Right (Nose Right)
                M_front_lat = (FY_veh_lf + FY_veh_rf) * front_to_CG
                M_rear_lat = -(FY_veh_lr + FY_veh_rr) * rear_to_CG  # Negative lever arm for rear
                
                M_front_long = (FX_veh_lf - FX_veh_rf) * (front_track_width / 2)
                M_rear_long = (FX_veh_lr - FX_veh_rr) * (rear_track_width / 2)
    
                MData[x, y, z] = M_front_lat + M_front_long + M_rear_lat + MZ_total
                

    return AyData, MData, slip_angle_range, steering_angle_range, slip_ratio_range


def plot_ymd(AyData, MData, SA_range, Delta_range, SX_range, save_path=None):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111)
    SA_len = len(SA_range)
    Delta_len = len(Delta_range)
    SX_len = len(SX_range)
    
    for y in range(Delta_len):
        for z in range(SX_len):
            Ay_SALine = AyData[:, y, z]
            M_SALine = MData[:, y, z] 
            ax.plot(Ay_SALine, M_SALine, color='r', linewidth=0.5, alpha=0.5)

    for x in range(SA_len):
        for z in range(SX_len):
            Ay_deltaLine = AyData[x, :, z]
            M_deltaLine = MData[x, :, z] 
            ax.plot(Ay_deltaLine, M_deltaLine, color='b', linewidth=0.5, alpha=0.5)

    ax.set_xlabel('Lateral Acceleration [G]')
    ax.set_ylabel('Yaw Moment [lb*ft]')
    ax.set_title('Yaw Moment Diagram')
    ax.grid(True)
    ax.axhline(0, color='black', linewidth=1)
    ax.axvline(0, color='black', linewidth=1)

    from matplotlib.lines import Line2D
    custom_lines = [Line2D([0], [0], color='r', lw=2),
                    Line2D([0], [0], color='b', lw=2)]
    ax.legend(custom_lines, ['Constant Steering', 'Constant Beta'])
    if save_path:
        plt.savefig(save_path)
        plt.close(fig)
    else:
        plt.show()

def plot_ymd_overlay(overlay_data, param_name, save_path=None):
    from matplotlib.lines import Line2D
    import matplotlib as mpl
    import matplotlib.colors as mcolors
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111)
    
    cmap = mpl.colormaps['viridis'] if hasattr(mpl, 'colormaps') else mpl.cm.get_cmap('viridis')
    norm = mcolors.Normalize(vmin=0, vmax=max(1, len(overlay_data)-1))
    
    custom_lines = []
    legend_labels = []
    
    limit_ay_points = []
    limit_mz_points = []
    
    for idx, data_tuple in enumerate(overlay_data):
        val, AyData, MData, SA_range, Delta_range, SX_range = data_tuple
        SA_len = len(SA_range)
        Delta_len = len(Delta_range)
        SX_len = len(SX_range)
        
        color = cmap(norm(idx))
        
        # Extract Limit Balance point: max Ay point
        Ay_flat = AyData[:, :, 0]
        Mz_flat = MData[:, :, 0]
        idx_max_ay = np.unravel_index(np.argmax(Ay_flat), Ay_flat.shape)
        limit_ay = Ay_flat[idx_max_ay]
        limit_mz = Mz_flat[idx_max_ay]
        
        limit_ay_points.append(limit_ay)
        limit_mz_points.append(limit_mz)
        
        # Plot constant steering lines
        for y in range(Delta_len):
            for z in range(SX_len):
                Ay_SALine = AyData[:, y, z]
                M_SALine = MData[:, y, z] 
                ax.plot(Ay_SALine, M_SALine, color=color, linewidth=0.5, alpha=0.7)

        # Plot constant beta lines
        for x in range(SA_len):
            for z in range(SX_len):
                Ay_deltaLine = AyData[x, :, z]
                M_deltaLine = MData[x, :, z] 
                ax.plot(Ay_deltaLine, M_deltaLine, color=color, linewidth=0.5, alpha=0.7)
                
        # Plot marker at limit balance
        ax.plot(limit_ay, limit_mz, marker='o', color=color, markersize=5)
                
        custom_lines.append(Line2D([0], [0], color=color, lw=2))
        legend_labels.append(f"{param_name} = {val:.4g}")

    # Plot line connecting limit balances
    if len(overlay_data) > 1:
        ax.plot(limit_ay_points, limit_mz_points, color='red', linestyle='-', linewidth=2)
        custom_lines.append(Line2D([0], [0], color='red', linestyle='-', lw=2))
        legend_labels.append('Limit Balance Path')

    ax.set_xlabel('Lateral Acceleration [G]')
    ax.set_ylabel('Yaw Moment [lb*ft]')
    ax.set_title(f'Yaw Moment Diagram Overlay - Sweeping {param_name}')
    ax.grid(True)
    ax.axhline(0, color='black', linewidth=1)
    ax.axvline(0, color='black', linewidth=1)
    ax.legend(custom_lines, legend_labels, loc='center left', bbox_to_anchor=(1.05, 0.5))
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, bbox_inches='tight')
        plt.close(fig)
    else:
        plt.show()

if __name__ == "__main__":
    with open('base_params_SCR26.yaml', 'r') as f:
        params = yaml.safe_load(f)

    # Open GUI to edit parameters
    params, sweep_config = param_gui_sweep.edit_params_for_sweep(params)
    
    if params is None:
        print("Cancelled.")
        exit(0)

    if sweep_config['type'] == 'none':
        # Run base YMD once
        Ay, Mz, SA_r, Delta_r, SX_r = generate_ymd(params)
        kpis = extract_kpis(Ay, Mz, SA_r, Delta_r)
        
        import os
        from datetime import datetime
        folder_name = f"Results_YMD_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.makedirs(folder_name, exist_ok=True)
        
        with open(f"{folder_name}/ymd_config.yaml", "w") as f:
            yaml.dump({'base_params': params}, f)
            
        kpi_dict = {k: float(v) for k, v in kpis.items()}
        with open(f"{folder_name}/kpis.yaml", "w") as f:
            yaml.dump({'kpis': kpi_dict}, f)
            
        print("======== BASE KPIs ========")
        for k, v in kpis.items():
            print(f"{k}: {v:.4f}")
            
        plot_ymd(Ay, Mz, SA_r, Delta_r, SX_r, save_path=f"{folder_name}/YMD_Plot.png")
        print(f"Base YMD complete. Results saved in {folder_name}/")
        
    elif sweep_config['type'] == '1d':
        param1 = sweep_config['param1']
        start = sweep_config['start1']
        end = sweep_config['end1']
        steps = sweep_config['steps1']
        kpi_list = sweep_config['kpis']
        
        sweep_vals = np.linspace(start, end, steps)
        
        import os
        from datetime import datetime
        folder_name = f"Results_KPI_Sweep_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.makedirs(folder_name, exist_ok=True)
        
        with open(f"{folder_name}/sweep_config.yaml", "w") as f:
            yaml.dump({'base_params': params, 'sweep_settings': sweep_config}, f)
        
        results = {kpi: [] for kpi in kpi_list}
        
        for val in sweep_vals:
            p_copy = copy.deepcopy(params)
            if param1 == 'velocity':
                Ay, Mz, SA_r, Delta_r, SX_r = generate_ymd(p_copy, velocity_override=val)
            else:
                set_nested_val(p_copy, param1, val)
                Ay, Mz, SA_r, Delta_r, SX_r = generate_ymd(p_copy)
            
            kpis = extract_kpis(Ay, Mz, SA_r, Delta_r)
            for k in kpi_list:
                results[k].append(kpis[k])
                
        # Plot 1D line graphs
        for kpi in kpi_list:
            plt.figure(figsize=(8, 6))
            plt.plot(sweep_vals, results[kpi], marker='o')
            plt.xlabel(param1)
            plt.ylabel(kpi)
            plt.title(f"{kpi} vs {param1}")
            plt.grid(True)
            plt.savefig(f"{folder_name}/{kpi.replace(' ', '_')}_vs_{param1.replace('.', '_')}.png")
            plt.close()
            
        print(f"Sweep 1D complete. Results saved in {folder_name}/")

    elif sweep_config['type'] == '1d_overlay':
        param1 = sweep_config['param1']
        start = sweep_config['start1']
        end = sweep_config['end1']
        steps = sweep_config['steps1']
        kpi_list = sweep_config['kpis']
        
        sweep_vals = np.linspace(start, end, steps)
        
        import os
        from datetime import datetime
        folder_name = f"Results_YMD_Overlay_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.makedirs(folder_name, exist_ok=True)
        
        with open(f"{folder_name}/sweep_config.yaml", "w") as f:
            yaml.dump({'base_params': params, 'sweep_settings': sweep_config}, f)
        
        results = {kpi: [] for kpi in kpi_list}
        overlay_data = []
        
        for val in sweep_vals:
            p_copy = copy.deepcopy(params)
            if param1 == 'velocity':
                Ay, Mz, SA_r, Delta_r, SX_r = generate_ymd(p_copy, velocity_override=val)
            else:
                set_nested_val(p_copy, param1, val)
                Ay, Mz, SA_r, Delta_r, SX_r = generate_ymd(p_copy)
            
            kpis = extract_kpis(Ay, Mz, SA_r, Delta_r)
            for k in kpi_list:
                results[k].append(float(kpis[k]))
                
            overlay_data.append((val, Ay, Mz, SA_r, Delta_r, SX_r))
            
        with open(f"{folder_name}/kpi_results.yaml", "w") as f:
            yaml.dump({'sweep_values': sweep_vals.tolist(), 'kpi_results': results}, f)
            
        plot_ymd_overlay(overlay_data, param1, save_path=f"{folder_name}/YMD_Overlay_Plot.png")
            
        for kpi in kpi_list:
            plt.figure(figsize=(8, 6))
            plt.plot(sweep_vals, results[kpi], marker='o')
            plt.xlabel(param1)
            plt.ylabel(kpi)
            plt.title(f"{kpi} vs {param1}")
            plt.grid(True)
            plt.savefig(f"{folder_name}/{kpi.replace(' ', '_')}_vs_{param1.replace('.', '_')}.png")
            plt.close()
            
        print(f"Overlay YMD complete. Results saved in {folder_name}/")

    elif sweep_config['type'] == '2d':
        param1 = sweep_config['param1']
        start1 = sweep_config['start1']
        end1 = sweep_config['end1']
        steps1 = sweep_config['steps1']
        
        param2 = sweep_config['param2']
        start2 = sweep_config['start2']
        end2 = sweep_config['end2']
        steps2 = sweep_config['steps2']
        
        kpi_list = sweep_config['kpis']
        
        p1_vals = np.linspace(start1, end1, steps1)
        p2_vals = np.linspace(start2, end2, steps2)
        
        P1, P2 = np.meshgrid(p1_vals, p2_vals)
        
        results = {kpi: np.zeros((steps2, steps1)) for kpi in kpi_list}
        
        import os
        from datetime import datetime
        folder_name = f"Results_KPI_Sweep_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.makedirs(folder_name, exist_ok=True)
        
        with open(f"{folder_name}/sweep_config.yaml", "w") as f:
            yaml.dump({'base_params': params, 'sweep_settings': sweep_config}, f)
        
        for i in range(steps2):
            for j in range(steps1):
                val1 = P1[i, j]
                val2 = P2[i, j]
                
                p_copy = copy.deepcopy(params)
                
                # Apply param 1
                if param1 == 'velocity':
                    v_ov = val1
                else:
                    v_ov = None
                    set_nested_val(p_copy, param1, val1)
                    
                # Apply param 2
                if param2 == 'velocity':
                    v_ov = val2
                else:
                    set_nested_val(p_copy, param2, val2)
                
                Ay, Mz, SA_r, Delta_r, SX_r = generate_ymd(p_copy, velocity_override=v_ov)
                kpis = extract_kpis(Ay, Mz, SA_r, Delta_r)
                
                for k in kpi_list:
                    results[k][i, j] = kpis[k]
                    
        # Plot 2D contours
        for kpi in kpi_list:
            plt.figure(figsize=(8, 6))
            cp = plt.contourf(P1, P2, results[kpi], cmap='viridis', levels=20)
            plt.colorbar(cp, label=kpi)
            plt.xlabel(param1)
            plt.ylabel(param2)
            plt.title(f"{kpi} Contour")
            plt.savefig(f"{folder_name}/{kpi.replace(' ', '_')}_contour.png")
            plt.close()
            
        print(f"Sweep 2D complete. Results saved in {folder_name}/")

