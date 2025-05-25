import numpy as np

# Constants and Parameters
# Projectile Properties (Attempting to model M107 projectile for M109A2)
m = 43.2  # mass (kg) - M107 projectile
d = 0.155   # reference diameter (m) - 155mm
# Moments of inertia for M107 (estimated, as precise values are hard to find)
# A_axial_approx = 0.5 * m * (d/2)^2 for solid cylinder: ~0.13 kg*m^2
# B_transverse_approx = 0.25 * m * (d/2)^2 + (1/12) * m * L^2 (L~0.605m): ~1.38 kg*m^2
# These are rough estimates; actual values depend on internal mass distribution.
A_axial = 0.13  # axial moment of inertia (kg*m^2) - Estimated for M107
B_transverse = 1.38  # transverse moment of inertia (kg*m^2) - Estimated for M107

# Ballistic Coefficient C_ballistic is NOT directly used in the current drag model if KD_0 is used.
# It was a placeholder. The primary drag is now from KD_0 and KD_a.
C_ballistic = 20.0 # Placeholder, higher value for less aero efficiency if it were used. True BC for M107 is complex.

# Aerodynamic Coefficients (placeholders - these are typically functions of Mach number)
# These are NOT specific to M107 and are major simplifications.
# For accurate M107 simulation, these would need to be replaced with data specific to M107.
KD_0 = 0.28  # Drag coefficient at zero yaw (increased slightly as M107 is not highly aerodynamic)
KD_a = 0.1  # Yaw drag force coefficient (placeholder)
KL = 0.05   # Lift force coefficient (placeholder)
KM = 0.01   # Pitching moment coefficient (used in ae) (placeholder)
KF = 0.005  # Magnus force coefficient (placeholder)
KT = 0.001  # Magnus moment coefficient (used in ae) (placeholder)
KN = 0.002  # Spin damping moment coefficient for roll (used in ae) (placeholder)
KA = 0.001  # Spin damping moment coefficient for axial spin (dN/dt) (placeholder)

# Launch Conditions
g0 = 9.80665  # acceleration due to gravity at launch (m/s^2)
launch_latitude = np.deg2rad(45.0)  # Launch latitude (radians)
launch_azimuth = np.deg2rad(90.0)  # Launch azimuth (radians from North, positive East)

# Earth Properties
R_earth = 6371000  # Earth radius (m)
Omega_earth = 7.2921159e-5  # Earth's angular velocity (rad/s)

# Simulation Parameters
dt = 0.01  # time step (s)
DEG_TO_MILS = 6400.0 / 360.0

# Initial Conditions (placeholders, will be set in the main simulation)
X0 = np.array([0.0, 0.0, 0.0]) # Initial position [x, y, z] (m) - z is altitude
# Ground-fixed coordinate system: 1-axis downrange, 2-axis vertical (initially parallel to g), 3-axis cross-range
# However, BRL report uses '2' axis parallel to g, so we'll adapt.
# Let's use: x (downrange), y (cross-range), z (altitude)
# So, gravity will be primarily in the -z direction.
# The report states "the 2-axis is parallel to g". We'll use z as the vertical axis.
# And we'll make g point in -z.
# u0_magnitude = 800.0 # m/s
# u0_elevation = np.deg2rad(45.0) # radians
# u0_azimuth_sim = np.deg2rad(0.0) # Azimuth for velocity vector in sim coords (0 along x-axis)
# u0 = np.array([u0_magnitude * np.cos(u0_elevation) * np.cos(u0_azimuth_sim),
#                u0_magnitude * np.cos(u0_elevation) * np.sin(u0_azimuth_sim),
#                u0_magnitude * np.sin(u0_elevation)])
N0 = 1386.0 # Initial axial spin (rad/s) (e.g., ~220 rev/s for M107 from M109A2, estimated)
ae0 = np.array([0.0, 0.0, 0.0]) # Initial yaw of repose vector

# Air Density Model (can be made more complex, e.g., ISA model)
def get_air_density(altitude):
    """
    Calculates air density based on altitude.
    Using a simple exponential model for now.
    rho = rho_0 * exp(-h/H)
    rho_0: sea level air density (kg/m^3)
    h: altitude (m)
    H: scale height (m)
    """
    rho_0 = 1.225  # kg/m^3
    H = 8500  # m
    return rho_0 * np.exp(-altitude / H)

# Force of Gravity (Equation 3.7 concept)
def get_gravity_vector(position, g0_val=g0, R0=R_earth):
    """
    Calculates the gravity vector.
    Approximation from Eq (3.7) for magnitude.
    Direction is towards Earth's center.
    In our ground-fixed system (x,y,z with z up), g is approx [0,0,-|g|].
    If origin is on surface, r = R0 + z.
    """
    r_vec_from_center = np.array([position[0], position[1], R0 + position[2]]) # Assuming launch point at (0,0,0) on surface for r calc
    r_mag_from_center = np.linalg.norm(r_vec_from_center)
    
    # Ensure r_mag_from_center is not zero to avoid division by zero
    if r_mag_from_center == 0:
        # This case should not happen in a typical trajectory from surface
        g_magnitude = g0_val 
    else:
        g_magnitude = g0_val * (R0 / r_mag_from_center)**2
        
    # Direction towards Earth's center. For a local cartesian frame with z up,
    # and assuming the origin is (0,0,0) on the surface, a point mass earth would
    # pull towards (0,0,-R_earth). So the unit vector is approx. -position_norm if origin is earth center.
    # For local z-up, the gravity vector is primarily [0,0,-g_magnitude].
    # The BRL report assumes "the 2-axis is parallel to g". If our z-axis is altitude, then g is [0,0,-g_magnitude].
    # For simplicity, we'll use a constant g vector pointing downwards along z.
    # A more precise model would account for Earth's curvature relative to the flat plane.
    # The report mentions "In the ground-fixed Cartesian system... the 2-axis is taken to be parallel to g".
    # If '2' is our z-axis (altitude), then g = [0, 0, -g_magnitude].
    return np.array([0.0, 0.0, -g_magnitude])


# Coriolis Acceleration (Equation 3.8)
def get_coriolis_acceleration(velocity_ground, lat=launch_latitude, az_launch=launch_azimuth, omega_e=Omega_earth):
    """
    Calculates Coriolis acceleration vector A.
    velocity_ground: u = [u1, u2, u3] (projectile velocity relative to ground)
                     In our chosen system: u1=vx, u2=vy, u3=vz
    lat: Launch latitude (phi in the report, L here)
    az_launch: Launch azimuth (AZ from report - this is for the *launch direction*, used to orient Q_earth components)
               The report's A vector uses u1,u2,u3 in the ground-fixed system.
               The components of Omega_earth in the ground-fixed system depend on the orientation
               of this system. If 1-axis is East, 2-axis North, 3-axis Up (ENU):
               Omega_vec = [0, omega_e * cos(lat), omega_e * sin(lat)]
               A = 2 * (u X Omega_vec)

               BRL Report No. 1314 uses a ground-fixed system where:
               - 1-axis is in the direction of launch azimuth
               - 3-axis is orthogonal to 1-axis in the horizontal plane (to the right)
               - 2-axis is vertical (parallel to g, positive upwards if g is downwards)

               Let's re-evaluate the Coriolis components based on this BRL coordinate system.
               Omega_earth vector components in this system:
               Omega_x_launch = omega_e * np.cos(lat) * np.sin(az_launch)  (Component along the 3-axis, cross-range right)
               Omega_y_launch = omega_e * np.sin(lat)                     (Component along the 2-axis, vertical)
               Omega_z_launch = omega_e * np.cos(lat) * np.cos(az_launch)  (Component along the 1-axis, downrange)
               
               The report provides A directly (eq 3.8):
               A1 = 2 * Omega_earth * (u3 * np.cos(lat) * np.sin(az_launch) - u2 * np.sin(lat))
               A2 = 2 * Omega_earth * (u1 * np.sin(lat) - u3 * np.cos(lat) * np.cos(az_launch))
               A3 = 2 * Omega_earth * (u2 * np.cos(lat) * np.cos(az_launch) - u1 * np.cos(lat) * np.sin(az_launch))
               
               If our system is x (downrange), y (cross-range right), z (vertical up)
               u = [ux, uy, uz]
               Then: u1 -> ux, u2 -> uz, u3 -> uy (Careful with axis mapping)

               Let's assume our (x,y,z) corresponds to the report's (1,3,2) axes:
               x -> 1-axis (downrange, direction of AZ)
               y -> 3-axis (cross-range, to the right of AZ)
               z -> 2-axis (vertical, up)

               So: u1 (report) = ux (our code)
                    u2 (report) = uz (our code)
                    u3 (report) = uy (our code)

               A_x = 2 * omega_e * (uy * np.cos(lat) * np.sin(az_launch) - uz * np.sin(lat))
               A_y = 2 * omega_e * (uz * np.cos(lat) * np.cos(az_launch) - ux * np.cos(lat) * np.sin(az_launch)) # This is A3 in report
               A_z = 2 * omega_e * (ux * np.sin(lat) - uy * np.cos(lat) * np.cos(az_launch)) # This is A2 in report
    """
    ux, uy, uz = velocity_ground[0], velocity_ground[1], velocity_ground[2]

    # Components of Omega_earth in the ground-fixed system (1-downrange, 3-crossrange right, 2-vertical up)
    # Omega_1 = omega_e * np.cos(lat) * np.cos(az_launch) # Along 1-axis (our x)
    # Omega_2 = omega_e * np.sin(lat)                     # Along 2-axis (our z)
    # Omega_3 = omega_e * np.cos(lat) * np.sin(az_launch) # Along 3-axis (our y)

    # A = 2 * (u x Omega_earth_vec_in_sim_frame)
    # Omega_earth_vec_sim = np.array([Omega_1, Omega_3, Omega_2]) # [Omega_x, Omega_y, Omega_z]
    
    # Using the report's direct equations (Eq 3.8) with mapping:
    # Report: (u1, u2, u3), (A1, A2, A3)
    # Our sim: (ux, uz, uy) for (u1, u2, u3) when comparing to report formula for A.
    # Our sim: (Ax, Az, Ay) for (A1, A2, A3) when comparing to report formula for A.
    
    # Let's use the standard vector cross product A = 2 * cross(u, Omega_sim)
    # Omega_earth vector in a frame: East, North, Up (ENU)
    # Omega_ENU = np.array([0, omega_e * np.cos(lat), omega_e * np.sin(lat)])
    
    # Transformation from ENU to our simulation frame (x-downrange at az_launch, y-crossrange right, z-up)
    # Rotation matrix from ENU to Sim:
    # Sim_x = North*cos(az_launch) + East*sin(az_launch)
    # Sim_y = -North*sin(az_launch) + East*cos(az_launch) (This makes y to the *left* if az is from North. Report's 3-axis is to the right)
    # Let's adjust az_launch for standard rotation: az_std = pi/2 - az_launch (if az_launch is from N)
    # Sim_x = E*cos(az_std) + N*sin(az_std)
    # Sim_y = -E*sin(az_std) + N*cos(az_std)
    # Sim_z = U
    
    # If az_launch is from x-axis (East) towards y-axis (North)
    # If az_launch is from North, clockwise:
    # Sim_x_axis_in_ENU = [sin(az_launch), cos(az_launch), 0]
    # Sim_y_axis_in_ENU = [cos(az_launch), -sin(az_launch), 0] (This is to the right of launch azimuth)
    # Sim_z_axis_in_ENU = [0, 0, 1]

    # Omega_sim_x = np.dot(Omega_ENU, Sim_x_axis_in_ENU) = omega_e * np.cos(lat) * np.cos(az_launch)
    # Omega_sim_y = np.dot(Omega_ENU, Sim_y_axis_in_ENU) = omega_e * np.cos(lat) * np.sin(az_launch) # Error in sign here before
    # Omega_sim_z = np.dot(Omega_ENU, Sim_z_axis_in_ENU) = omega_e * np.sin(lat)

    # Correct components of Omega_earth in simulation frame (x-downrange, y-crossrange right, z-up):
    Omega_sim_x = omega_e * np.cos(lat) * np.cos(az_launch) # North component projected on x
    Omega_sim_y = omega_e * np.cos(lat) * np.sin(az_launch) # East component projected on y (azimuth from North to East)
    Omega_sim_z = omega_e * np.sin(lat)                     # Up component is z

    # Wait, az_launch is azimuth *of launch*.
    # The report's system (1,2,3) is: 1-axis along launch azimuth, 2-axis vertical, 3-axis to the right.
    # Omega components in (1,2,3) frame from report:
    # Omega_1_report = omega_e * cos(lat) * cos(AZ)
    # Omega_2_report = omega_e * sin(lat)
    # Omega_3_report = -omega_e * cos(lat) * sin(AZ) (Note the minus sign)

    # Our (x,y,z) is (1,3,2) of report.
    # Omega_x_ours = Omega_1_report = omega_e * np.cos(lat) * np.cos(az_launch)
    # Omega_y_ours = Omega_3_report = -omega_e * np.cos(lat) * np.sin(az_launch)
    # Omega_z_ours = Omega_2_report = omega_e * np.sin(lat)
    
    Omega_vec_sim_frame = np.array([
        omega_e * np.cos(lat) * np.cos(az_launch),       # Component along x (downrange)
        -omega_e * np.cos(lat) * np.sin(az_launch),      # Component along y (cross-range right)
        omega_e * np.sin(lat)                            # Component along z (vertical up)
    ])
    
    A_coriolis = 2 * np.cross(velocity_ground, Omega_vec_sim_frame)
    return A_coriolis

# Velocity relative to air (Equation 3.4)
def get_velocity_relative_to_air(velocity_ground, wind_velocity=np.array([0.0, 0.0, 0.0])):
    """
    Calculates velocity relative to air.
    v = u - w
    """
    return velocity_ground - wind_velocity

# Core Equation Implementations

# Equation 3.2: Axial Spin Update
def get_dN_dt(rho, d_ref, A_axial_inertia, KA_spin_damping, N_spin, v_airspeed_mag):
    """
    Calculates the rate of change of axial spin dN/dt.
    dN/dt = - (rho * d^4 / (2 * A_axial_inertia)) * KA_spin_damping * N_spin * v_airspeed_mag
    
    Args:
        rho: Air density (kg/m^3)
        d_ref: Reference diameter (m)
        A_axial_inertia: Axial moment of inertia (kg*m^2)
        KA_spin_damping: Spin damping moment coefficient
        N_spin: Axial spin (rad/s)
        v_airspeed_mag: Magnitude of velocity relative to air (m/s)
    Returns:
        float: dN/dt (rad/s^2)
    """
    if A_axial_inertia == 0: # Avoid division by zero
        return 0.0
    dN_dt = - (rho * d_ref**4) / (2 * A_axial_inertia) * KA_spin_damping * N_spin * v_airspeed_mag
    return dN_dt

# Equation 3.3: Estimate for Yaw of Repose (ae)
# This is a complex equation. We will start by defining the coefficients
# a_alpha, a_omega, a_g, a_L, and then the equation for ae.
# The report mentions simplified forms. For now, let's structure the full form.
# Denominators D1 and D2 are mentioned, which can simplify.

def get_yaw_of_repose_coefficients(m_mass, d_ref, B_trans_inertia, N_spin, v_airspeed_mag, rho,
                                   KL_lift, KM_pitch_moment, KF_magnus_force, KT_magnus_moment, KN_spin_damping_roll):
    """
    Calculates the coefficients a_alpha, a_omega, a_g, a_L for the yaw of repose equation (3.3).
    These are defined below Eq. (3.3) in the BRL report.

    Note: The report mentions simplifications for D1 and D2 (denominators).
    D1 = (B * N / v) - (rho * d^2 / (2*m)) * KL * v
    D2 = (B * N / v) - (rho * d^4 / (2*B)) * KM * v  -- Error in my transcription, should be d^5 and different KM, likely K_Mq + K_M_alpha_dot

    Let's refer to the definitions in the report carefully:
    a_alpha_numerator = (rho * d^3 / (2*B)) * KT
    a_omega_numerator = 1.0
    a_g_numerator = (m * d / B)
    a_L_numerator = (rho * d^2 / (2*B)) * (KT - KN) -- Simpler form used: (rho * d^4 / (2*B*v)) * (C_M_q + C_M_alpha_dot) type terms often appear here.
                                               -- Report has a_L based on D2, which is (B N / v) - (rho d^4 / (2B)) * K_M v where K_M is pitching moment coeff.
                                               -- Let's re-check source for a_L.
                                               -- Eq 2.19 shows terms. Eq 2.20 is simplified ae.
    The definitions just below (3.3) are:
    a_alpha = [(rho d^3)/(2B)] K_T / D2
    a_omega = 1 / D1
    a_g     = (md/B) / D1
    a_L     = [(rho d^2)/(2B)] (K_T - K_N) / D1  -- This looks more plausible.

    Where:
    D1 = (N/v) - (rho d^2 v / (2m)) K_L
    D2 = (N/v) - (rho d^4 v / (2B)) K_M_prime  (K_M_prime is K_M_p_alpha_dot + K_M_q from other texts, related to dynamic stability)
               The report uses K_M (pitching moment coefficient) for K_M_prime in D2.
    """
    if v_airspeed_mag < 1e-6: # Avoid division by zero for v
        return 0.0, 0.0, 0.0, 0.0

    # Denominators D1 and D2 as per definitions below Eq (3.3)
    # D1 = (N_spin / v_airspeed_mag) - (rho * d_ref**2 * v_airspeed_mag / (2 * m_mass)) * KL_lift
    # D2 = (N_spin / v_airspeed_mag) - (rho * d_ref**4 * v_airspeed_mag / (2 * B_trans_inertia)) * KM_pitch_moment

    # Simpler denominators from (2.18) where terms are neglected:
    D1_simple = N_spin / v_airspeed_mag
    D2_simple = N_spin / v_airspeed_mag
    
    # Using the D1, D2 from below (3.3) for now, assuming B_trans_inertia !=0 and m_mass !=0
    val_D1 = (N_spin / v_airspeed_mag) - (rho * d_ref**2 * v_airspeed_mag / (2 * m_mass)) * KL_lift
    val_D2 = (N_spin / v_airspeed_mag) - (rho * d_ref**4 * v_airspeed_mag / (2 * B_trans_inertia)) * KM_pitch_moment

    # Avoid division by zero for D1, D2
    epsilon = 1e-9 # Small number to prevent division by zero
    if abs(val_D1) < epsilon: val_D1 = np.sign(val_D1) * epsilon if val_D1 !=0 else epsilon
    if abs(val_D2) < epsilon: val_D2 = np.sign(val_D2) * epsilon if val_D2 !=0 else epsilon


    a_alpha = ((rho * d_ref**3) / (2 * B_trans_inertia)) * KT_magnus_moment / val_D2
    a_omega = 1.0 / val_D1
    a_g = (m_mass * d_ref / B_trans_inertia) / val_D1
    a_L = ((rho * d_ref**2) / (2 * B_trans_inertia)) * (KT_magnus_moment - KN_spin_damping_roll) / val_D1
    
    return a_alpha, a_omega, a_g, a_L

def get_yaw_of_repose_vector_ae(v_air_vec, ae_prev, N_spin, g_vec,
                                m_mass, d_ref, B_trans_inertia, rho,
                                KL_lift, KM_pitch_moment, KF_magnus_force, KT_magnus_moment, KN_spin_damping_roll,
                                use_simplified_ae=False):
    """
    Calculates the yaw of repose vector 'ae' using Equation (3.3) or simplified (2.20).
    ae = a_alpha * (v_air_vec X ae_prev) + a_omega * (ae_prev X x_axis_projectile) + \
         a_g * (x_axis_projectile X g_vec) + a_L * (I_trans_vec X v_air_vec)

    Args:
        v_air_vec: Velocity vector relative to air.
        ae_prev: Previous yaw of repose vector.
        N_spin: Axial spin (rad/s).
        g_vec: Gravity vector.
        m_mass, d_ref, B_trans_inertia: Projectile properties.
        rho: Air density.
        KL_lift, ..., KN_spin_damping_roll: Aerodynamic coefficients.
        use_simplified_ae: If True, uses simplified Eq (2.20) if available/implemented.

    Returns:
        np.array: Yaw of repose vector ae.
    """
    v_airspeed_mag = np.linalg.norm(v_air_vec)
    if v_airspeed_mag < 1e-6:
        return np.array([0.0, 0.0, 0.0]) # No velocity, no significant aerodynamic yaw

    # Projectile's x-axis (axis of symmetry) is assumed to be aligned with v_air_vec for this simplified model
    # This is an approximation. In reality, x_axis_projectile would evolve with projectile orientation.
    # For the modified point mass model, ae represents the angle between v_air_vec and the projectile's axis.
    # The term (ae_prev X x_axis_projectile) means ae is perpendicular to x_axis.
    # The vector 'x' in 'ae X x' and 'x X g' (from Eq 3.3) is the unit vector along the projectile's axis of symmetry.
    # In the MPM model, it's often assumed that the projectile axis quickly aligns *near* the velocity vector.
    # A common simplification is to consider x_axis_projectile approx. parallel to v_air_vec / |v_air_vec|.
    
    # If we take x_axis_projectile as v_air_vec_unit:
    x_axis_projectile_unit = v_air_vec / v_airspeed_mag

    # The term I_trans_vec for a_L * (I_trans_vec X v_air_vec):
    # I is the unit vector along the transverse inertial axis. This is not well-defined without full orientation.
    # The report's derivation of (3.3) shows it as (I_axis X v_unit).
    # The source indicates "I is a unit vector normal to x and v". This implies I is related to the yaw plane.
    # If ae represents the yaw (angle between v and x_projectile), then I could be (x_projectile X v_unit) / |x_projectile X v_unit|.
    # This term a_L * (I X v) seems to represent a lift-like effect due to yaw.
    # Let's re-check derivation of (3.3) or (2.19) for 'I'.
    # In (2.19), the term is L_vector / (B D1). L_vector involves (K_T - K_N) * (v x (v x k_y)) where k_y is transverse axis.
    # This is complex.
    # The simplified form (2.20) might be more tractable:
    # ae_simple = ( (m*d) / (B*N/v) ) * (x_proj X g_vec) + ( (rho*d^2*v) / (2*B*N/v) ) * (K_L_alpha * (v_vec X x_proj) + K_M_alpha * x_proj)
    # This requires K_L_alpha (lift curve slope) and K_M_alpha (pitching moment curve slope).
    # Our KL is likely K_L_alpha. KM is K_M_alpha.

    if use_simplified_ae: # Placeholder for simplified version (e.g. Eq 2.20)
        # Simplified version (2.20) from BRL 1314 is:
        # ae = ( (m*d*v_mag) / (B_trans_inertia*N_spin) ) * np.cross(x_axis_projectile_unit, g_vec)  + \
        #      ( (rho*d_ref**2*v_mag**2) / (2*B_trans_inertia*N_spin) ) * \
        #      ( KL_lift * np.cross(v_air_vec, x_axis_projectile_unit) + KM_pitch_moment * x_axis_projectile_unit)
        # Note: The term KL_lift * cross(v,x) in 2.20 is problematic if x is parallel to v.
        # The intent is that K_L_alpha * alpha where alpha is angle.
        # The first term (gravity term) in 2.20 induces a component of ae.
        # The second term (aerodynamic term) includes lift and moment.
        # If x_axis_projectile_unit is approx v_air_vec/v_airspeed_mag, then cross(v_air_vec, x_axis_projectile_unit) is zero.
        # This suggests K_L in 2.20 might be different or my x_axis_projectile_unit assumption is too simple for 2.20.

        # For now, let's assume the simplified form where ae is primarily due to gravity effect (first term of 2.20)
        # And neglect the complex aerodynamic part of 2.20 or assume it's small.
        # Or, if ae_prev is small, the (v X ae) and (ae X x) terms in (3.3) are also complex.
        
        # Given the complexity, let's try the structure of (3.3) with approximations or default to zero if too unstable.
        # The paper states: "ae is the 'yaw of repose' vector, an estimate of the average yaw of the projectile"
        # "For initial computations, ae may be taken as zero." (Section 3.e)
        # This might be the best starting point if coefficients are uncertain.
        # For now, returning zero as a robust placeholder until coefficients are well-defined.
        # If we must implement (3.3), we need to be careful.
        # The term (ae X x) implies x is the projectile axis. If ae itself defines deviation of x from v, it's circular.
        # Eq (3.3) is an equation *for* ae. ae_prev is from previous time step.
        
        # Fallback to a very simple gravity-induced yaw if N_spin is high:
        if abs(N_spin) > 1e-3 and B_trans_inertia > 1e-3:
             ae_g_term_coeff = (m_mass * d_ref * v_airspeed_mag) / (B_trans_inertia * N_spin)
             ae_simple = ae_g_term_coeff * np.cross(x_axis_projectile_unit, g_vec)
             # print(f"Simplified ae_g_term: {ae_simple}")
             return ae_simple
        else:
             return np.array([0.0, 0.0, 0.0])


    a_alpha, a_omega, a_g, a_L = get_yaw_of_repose_coefficients(
        m_mass, d_ref, B_trans_inertia, N_spin, v_airspeed_mag, rho,
        KL_lift, KM_pitch_moment, KF_magnus_force, KT_magnus_moment, KN_spin_damping_roll
    )

    # If coefficients are zero (e.g., due to low speed or zero N_spin in denominators), ae might be zero.
    if all(abs(c) < 1e-9 for c in [a_alpha, a_omega, a_g, a_L]):
        return np.array([0.0, 0.0, 0.0])

    # Term definitions for Eq (3.3)
    # ae = a_alpha * (v X ae_prev) + a_omega * (ae_prev X x) + a_g * (x X g) + a_L * (I X v)
    
    # Vector 'x' (projectile axis): In MPM, often approximated by v_air_vec/|v_air_vec| or related to ae_prev.
    # If ae_prev is the angle from v_air_vec to x_projectile_axis, then x_projectile_axis is not v_air_vec.
    # However, the model assumes small yaw (alpha). So x_projectile_axis is close to v_air_vec_unit.
    # Let's use x_axis_projectile_unit = v_air_vec / v_airspeed_mag as a first approximation for 'x'.
    
    term1 = a_alpha * np.cross(v_air_vec, ae_prev)
    term2 = a_omega * np.cross(ae_prev, x_axis_projectile_unit) # x is unit vector along shell axis
    term3 = a_g * np.cross(x_axis_projectile_unit, g_vec)
    
    # For term4: a_L * (I X v_air_vec)
    # "I is a unit vector normal to x and v" - BRL p.7
    # This means I is in the direction of np.cross(x_axis_projectile_unit, v_air_vec_unit), which is related to ae.
    # If x_axis_projectile_unit is approx v_air_vec_unit, then (I X v) would be small or zero.
    # This term might be better captured by a simplified model or if ae_prev defines the plane for I.
    # If ae = (alpha_complex_plane_vector), then I might be along ae_prev normalized, or perpendicular to it.
    # Given the note "I is a unit vector normal to x and v", and if x is slightly different from v,
    # let I be in the direction of ae, assuming ae is small and represents this deviation.
    # So I_unit could be ae_prev / norm(ae_prev) if norm(ae_prev) > epsilon.
    # This interpretation might be circular or incorrect.
    # The source (pg. 7, under eq 2.19 discussion) links the a_L term to (K_T - K_N).
    # The formulation of a_L itself uses (KT - KN).
    # The term is a_L * ( (ae_prev/|ae_prev|) X v_air_vec ) if I is along ae_prev. This seems like a damping term.

    # Let's check Eq (2.19) which is the basis for (3.3).
    # The last term in (2.19) for d(alpha_vector)/dt is:
    # ( (rho d^2 v) / (2B) ) * (K_T - K_N) * ( (v/|v|) X ( (v/|v|) X k_y ) ) / D1
    # where k_y is a transverse axis unit vector. (v/|v|) X k_y is a vector in yaw plane.
    # (v/|v|) X ( (v/|v|) X k_y ) = -k_y_perp_to_v  (vector rejection of k_y from v_dir)
    # This simplifies to effectively -k_y if k_y is already perp to v.
    # This term seems to try to align k_y with something or damp its motion.
    #
    # For (3.3), the term is a_L * (I X v). If I is in yaw plane, normal to x and v,
    # I could be taken as proportional to np.cross(x_axis_projectile_unit, v_air_vec_unit).
    # But that's zero if x_axis_projectile_unit = v_air_vec_unit.
    # If ae_prev represents the yaw (vector from tip of v_air_vec to tip of x_proj_scaled),
    # then a vector in the yaw plane, normal to v_air_vec, could be ae_prev - np.dot(ae_prev, v_air_vec_unit)*v_air_vec_unit.
    # Let's simplify or assume this term is small for now due to ambiguity of 'I'.
    # The report notes that for "initial computations, ae may be taken as zero".
    # If ae_prev is zero, then term1 and term2 are zero.
    # If ae_prev is zero, only term3 (gravity) and term4 (lift/Magnus moment related) would contribute to generating initial ae.
    #
    # Consider the simplified ae from (2.20) again for insight into the a_L like term.
    # It had (rho d^2 v^2 / (2BN)) * [ K_L_alpha * (v X x_unit) + K_M_alpha * x_unit ].
    # The (v X x_unit) is related to ae. K_M_alpha * x_unit pushes x_unit.
    #
    # Let's assume for now the (I X v) term is modeled by the a_L coefficient acting on something simple or is negligible.
    # A common simplification in some trajectory models is to neglect the full complexity of ae dynamics
    # or use an equilibrium/repose condition.
    # The equation (3.3) IS the equation for dae/dt = ... where ae is the state.
    # No, (3.3) is ae = ... it's an algebraic relation for ae at each step, not a differential equation for ae.
    # This means ae is assumed to reach this equilibrium state quickly at each time step.

    # If ae_prev is small, (v X ae_prev) and (ae_prev X x) are first order in ae_prev.
    # (x X g) is zeroth order. (I X v) is also potentially zeroth order if I is defined independently of ae_prev.
    #
    # Given the BRL's advice: "For initial computations, ae may be taken as zero."
    # And then "In succeeding computations, ae as determined from the previous computation point may be used."
    # This implies ae is iteratively updated or used from previous step.
    # This contradicts ae being an algebraic solution *at the current step* based on current v, N, g etc.
    #
    # Let's look at the context of (3.3) in Section 3.d. "Estimate for Yaw of Repose".
    # It says ae "is determined by the expression (3.3)". This implies algebraic.
    #
    # Re-evaluating the term a_L * (I X v):
    # If 'I' is a unit vector in the yaw plane, perpendicular to the projectile's axis 'x',
    # and also perpendicular to 'v'.
    # This sounds like I = normalize(cross(x,v)). If x and v are nearly aligned, this is unstable.
    #
    # Alternative from a similar model (e.g. McCoy's "Modern Exterior Ballistics"):
    # The yaw of repose can often be linked to the swerving motion caused by gravity on a spinning projectile.
    # ae_gravity_component ~ (m*g*d / (B*N*v_inv)) * cross(x_unit, g_unit_perp_to_x)
    # This is similar to the a_g * (x X g) term.
    #
    # Let's assume for now that the definition of 'I' in a_L*(I X v) is ambiguous for MPM without full projectile orientation.
    # Many MPMs simplify ae significantly.
    #
    # Simplification: if ae_prev is small, terms with ae_prev might be initially less dominant than gravity term.
    # If we set term4 = 0 due to ambiguity of I:
    term4 = np.array([0.0,0.0,0.0]) # Placeholder for a_L * (I X v)

    ae_new = term1 + term2 + term3 + term4
    
    # Limit the magnitude of ae to prevent instability, e.g., to a few degrees (0.05 rad ~ 3 deg)
    max_ae_mag = 0.1 # about 5.7 degrees
    norm_ae_new = np.linalg.norm(ae_new)
    if norm_ae_new > max_ae_mag:
        ae_new = (ae_new / norm_ae_new) * max_ae_mag
        # print(f"Warning: Yaw of repose ae limited from {norm_ae_new:.3f} to {max_ae_mag:.3f}")

    return ae_new


# Equation 3.1: Equation of Motion for the Center of Mass
def get_acceleration_vector_u_dot(rho, C_ballistic, KD_drag, KD_a_yaw_drag,
                                  v_air_vec, d_ref, m_mass, KL_lift, ae_yaw_repose_vec,
                                  g_vec, A_coriolis_vec, KF_magnus, N_spin):
    """
    Calculates the acceleration vector u_dot (du/dt) for the center of mass using Equation (3.1).
    u_dot = - (rho / (144 * C)) * (KD + KD_a * Q) * v * v_vec + \
             (rho * d^2 / (2 * m)) * KL * v^2 * ae_vec + \
             g_vec + \
             A_coriolis_vec + \
             (rho * d^3 / (2 * m)) * KF * N * Q * (ae_vec X v_vec)
             
    Note: The report uses C for ballistic coeff, but units seem to be mass/area (like slug/ft^2 or kg/m^2).
          The first term has 144 * C. If C is ballistic coeff = m / (Cd * Area_ref) = m / (Cd_form * pi*d^2/4),
          then 1/(144C) seems like an old unit conversion factor.
          The BRL report states C = w / (i * d^2 * Cd_std), where w is weight, i is form factor, d is diameter, Cd_std is standard drag.
          This C has units of lb/ft^2. The factor 144 converts in^2 to ft^2.
          If we use SI units throughout (m, kg, s), C should be m / (i * Cd_std). (Cd_form * Area_ref) is effectively (i * Cd_std * d^2).
          So C (ballistic coeff) = m / (DragRefAreaFactor * d^2).
          The term is -(rho / (2*C_ballistic_SI)) * ... if C_ballistic_SI = m / (Cd_form * pi*d^2/4).
          
          Let's re-evaluate the drag term from a more standard form:
          F_drag = 0.5 * rho * v^2 * Cd_total * Area_ref
          a_drag = F_drag / m = (0.5 * rho * v^2 * Cd_total * Area_ref) / m
          Cd_total = (KD_base + KD_a * Q_yaw_drag_factor) using report notation.
          Area_ref = pi * (d_ref/2)^2 = (pi/4)*d_ref^2.
          a_drag = (0.5 * rho * v_mag * (KD_drag + KD_a_yaw_drag * Q_yaw_drag_factor) * (np.pi/4 * d_ref**2) / m_mass) * (-v_air_unit_vec)

          The report's first term: `Acc_drag = - (rho / (144 * C)) * (KD + KD_a * Q) * v_mag * v_air_vec`
          If C is ballistic coefficient in lb/in^2, (w_lb / d_in^2), and KD is dimensionless.
          The factor 144 * C would convert C to lb/ft^2 if C was in lb/in^2.
          C (ballistic coefficient) = m / (Cd_ref * A_ref). Here m is mass. Cd_ref is some reference drag coeff. A_ref is ref area.
          The term should be: - (0.5 * rho * v_mag * (KD_drag + KD_a_yaw_drag * Q_yaw_drag_factor) * A_ref / m_mass) * v_air_unit_vec
                          = - (rho * v_mag * (KD_drag + KD_a_yaw_drag * Q_yaw_drag_factor) * (np.pi/8 * d_ref**2) / m_mass) * v_air_unit_vec
          
          Comparing to report: (rho / (144*C_brl)) * v_mag vs (rho * (np.pi/8 * d_ref**2) / m_mass)
          So (1 / (144*C_brl)) must be equivalent to ( (np.pi/8 * d_ref**2) / m_mass ) / Cd_implicit
          where Cd_implicit = (KD_drag + KD_a_yaw_drag * Q_yaw_drag_factor).

          The C_ballistic parameter (kg/m^2) we defined: C_ballistic = m / (Cd_standard * Area_ref_standard)
          So, (1/C_ballistic) = (Cd_standard * Area_ref_standard) / m
          Drag accel = - 0.5 * rho * v_mag * ( (KD_drag + KD_a_yaw_drag*Q_yaw_drag_factor) / Cd_standard ) * ( (Cd_standard * Area_ref_standard) / m_mass ) * v_air_unit_vec
                     = - 0.5 * rho * v_mag * K_prime * (1/C_ballistic) * v_air_unit_vec
                     where K_prime is ratio of actual drag to standard drag used for C_ballistic.
          If KD_drag is the actual drag coefficient (like Cd_total_at_zero_yaw), and Area_ref = pi/4 * d_ref^2,
          Then Drag Accel = - (0.5 * rho * v_mag * (KD_drag + KD_a_yaw_drag * Q_yaw_drag_factor) * (np.pi/4 * d_ref**2) / m_mass) * v_air_unit_vec
          
          Let's use the explicit form with area:
          Area_ref = (np.pi / 4) * d_ref**2
          Drag_coeff_term = KD_drag + KD_a_yaw_drag * Q_yaw_drag_factor (where Q_yaw_drag_factor is derived from ae)
          term_drag_accel_mag = (0.5 * rho * Drag_coeff_term * Area_ref) / m_mass
    """
    v_airspeed_mag = np.linalg.norm(v_air_vec)
    if v_airspeed_mag < 1e-6: # if no airspeed, no aerodynamic forces essentially
        return g_vec + A_coriolis_vec # Only gravity and Coriolis if no air speed

    v_air_unit_vec = v_air_vec / v_airspeed_mag

    # Drag Term
    # Using the standard aerodynamic equation: F_drag = 0.5 * rho * v^2 * Cd * Area
    # a_drag = F_drag / m
    Area_ref = (np.pi / 4) * d_ref**2
    current_drag_coeff = KD_drag # This KD should be total drag coefficient at current conditions (e.g. C_D_0 in some notations)
    # The term (KD + KD_a * Q) from the report implies KD is zero-yaw drag and KD_a*Q is yaw-dependent drag.
    # Q (yaw drag factor) is not defined in the provided snippet. It's often related to sin^2(alpha_total_yaw).
    # For MPM, alpha_total_yaw is related to |ae|. So Q could be |ae|^2.
    # Let's assume Q_yaw_drag_factor is this |ae|^2 for now.
    # This is an assumption. The report might define Q differently.
    # Section 2.c.1: "Q is a factor which is a function of the yaw, usually Q = sin^2(alpha_T)"
    # alpha_T is the total yaw angle. In MPM, |ae| represents this. So Q = np.linalg.norm(ae_yaw_repose_vec)**2.
    
    Q_factor = np.linalg.norm(ae_yaw_repose_vec)**2 # Assuming Q = |ae|^2

    total_drag_coeff_factor = KD_drag + KD_a_yaw_drag * Q_factor
    
    # The report's drag term: - (rho / (144 * C_brl)) * total_drag_coeff_factor * v_airspeed_mag * v_air_vec
    # Where C_brl is the ballistic coefficient (e.g. m_slugs / d_ft^2).
    # The constant C_ballistic we have (kg/m^2) is m_kg / (Cd_ref * Area_m^2_ref).
    # If we assume the (KD_drag + KD_a_yaw_drag * Q_factor) is the effective Cd:
    # Accel_drag_mag = (0.5 * rho * v_airspeed_mag**2 * total_drag_coeff_factor * Area_ref) / m_mass
    # Accel_drag_vec = -Accel_drag_mag * v_air_unit_vec
    # This is: - (0.5 * rho * v_airspeed_mag * total_drag_coeff_factor * Area_ref / m_mass) * v_air_vec
    
    # Let's use the C_ballistic directly if the report structure is to be followed closely.
    # The first term in Eq (3.1) is: ` - (rho / (144 * C_brl)) * (KD + KD_a * Q) * v_mag * v_air_vec`
    # If our C_ballistic (kg/m^2) is intended to be C_brl * (conversion_factor_mass_kg_per_slug / conversion_factor_area_m2_per_ft2)
    # This path is confusing due to the 144 factor and old units.
    # Let's use the modern physics form: a = F/m.
    
    # Drag Force: Fd_vec = -0.5 * rho * v_mag^2 * total_drag_coeff_factor * Area_ref * v_air_unit_vec
    # Fd_vec = -0.5 * rho * v_mag * total_drag_coeff_factor * Area_ref * v_air_vec
    term1_drag = - (0.5 * rho * v_airspeed_mag * total_drag_coeff_factor * Area_ref / m_mass) * v_air_vec
    
    # Lift Term
    # F_lift = 0.5 * rho * v^2 * CL_total * Area_ref * lift_direction
    # The report has: (rho * d^2 / (2 * m)) * KL * v^2 * ae
    # Here, d^2 is not Area_ref. Area_ref = pi/4 * d^2.
    # So (rho * d^2 / (2m)) = (rho * (4/pi)*Area_ref / (2m)) = (rho * Area_ref * (2/pi) / m)
    # This implies their KL is related to CL by a factor of (pi/2). CL = KL * (pi/2) ?
    # Or, their KL is CL and d^2 is a reference length squared, not directly area.
    # Standard: F_L = 0.5 * rho * v^2 * C_L_alpha * alpha * Area_ref.
    # If ae is the yaw vector (small angle approx for alpha), then |ae| is alpha.
    # And ae / |ae| is the direction of yaw, lift is perpendicular to v and in plane of yaw.
    # Lift force direction vector: normalize(cross(cross(v_air_vec, ae_yaw_repose_vec), v_air_vec))
    # The report simplifies this to: Lift_Accel = (rho * d_ref**2 / (2 * m_mass)) * KL_lift * v_airspeed_mag**2 * ae_yaw_repose_vec
    # This means ae_yaw_repose_vec itself gives the direction and magnitude proportional to yaw for lift.
    # This is plausible if ae is defined appropriately as a "lift-generating yaw vector".
    term2_lift = (rho * d_ref**2 / (2 * m_mass)) * KL_lift * v_airspeed_mag**2 * ae_yaw_repose_vec

    # Gravity Term
    term3_gravity = g_vec

    # Coriolis Term
    term4_coriolis = A_coriolis_vec

    # Magnus Term (Side Force)
    # F_magnus = 0.5 * rho * v^2 * C_magnus * Area_ref * magnus_direction
    # C_magnus typically depends on spin parameter (N*d / v).
    # Report: (rho * d^3 / (2 * m)) * KF * N * Q * (ae X v)
    # KF is Magnus force coefficient. N is spin. Q is yaw factor.
    # Direction is (ae X v). This is different from typical (spin_axis X v) or ( (spin_axis X v_body) X v_body )
    # The term (ae X v) is perpendicular to v and ae.
    # If ae is small, this term suggests Magnus force related to yaw and spin.
    # This form of Magnus term is specific to this model.
    # Note: Q = |ae|^2. If ae is zero, Q is zero, Magnus is zero.
    # If ae is non-zero, (ae X v) is a vector.
    if abs(N_spin) > 1e-6 and Q_factor > 1e-9 : # requires spin and yaw
        term5_magnus = (rho * d_ref**3 / (2 * m_mass)) * KF_magnus * N_spin * Q_factor * np.cross(ae_yaw_repose_vec, v_air_vec)
    else:
        term5_magnus = np.array([0.0, 0.0, 0.0])
        
    # Total acceleration
    u_dot = term1_drag + term2_lift + term3_gravity + term4_coriolis + term5_magnus
    return u_dot


# Numerical Integration (RK4)
def rk4_step(f, y, t, dt, **kwargs):
    """
    Performs a single step of RK4 integration.
    y_next = y + (k1 + 2*k2 + 2*k3 + k4) / 6.0
    
    Args:
        f: Function that calculates the derivative (e.g., dy/dt = f(t, y, **kwargs)).
           It must return a NumPy array for the derivatives.
        y: Current state vector (NumPy array).
        t: Current time.
        dt: Time step.
        kwargs: Additional arguments required by the derivative function f.
                These are passed through to f.
    Returns:
        NumPy array: State vector at t + dt.
    """
    k1 = dt * f(t, y, **kwargs)
    k2 = dt * f(t + 0.5 * dt, y + 0.5 * k1, **kwargs)
    k3 = dt * f(t + 0.5 * dt, y + 0.5 * k2, **kwargs)
    k4 = dt * f(t + dt, y + k3, **kwargs)
    return y + (k1 + 2*k2 + 2*k3 + k4) / 6.0

# Derivative function for the system of ODEs
# State vector y = [x, y, z, ux, uy, uz, N, ae_x, ae_y, ae_z]
# However, ae is solved algebraically, not as a differential equation here.
# So state is y = [x_pos, y_pos, z_pos, u_ground_x, u_ground_y, u_ground_z, N_spin]
# d(ae)/dt is not directly given by eq 3.3, rather ae is an estimate based on current state.

def system_derivatives(t, state, current_ae_vec): # ae is passed in, not part of integrated state
    """
    Calculates derivatives for the projectile state vector.
    State vector: [x_pos, y_pos, z_pos, u_ground_x, u_ground_y, u_ground_z, N_spin]
    
    Args:
        t: Current time (not directly used by all physics equations but needed for RK4 structure).
        state: Current state [X_x, X_y, X_z, u_x, u_y, u_z, N].
        current_ae_vec: Current yaw of repose vector (calculated algebraically before this call).
    
    Returns:
        NumPy array: Derivatives [dX/dt, du/dt, dN/dt].
    """
    X_pos = state[0:3]
    u_ground = state[3:6]
    N_val = state[6]

    # 1. Air density
    rho_val = get_air_density(X_pos[2]) # z-coordinate is altitude

    # 2. Velocity relative to air (assuming no wind for now)
    # wind_vel = np.array([0.0, 0.0, 0.0]) # TODO: Make wind a parameter
    v_air = get_velocity_relative_to_air(u_ground, wind_velocity=np.array([0.0,0.0,0.0]))
    v_air_mag = np.linalg.norm(v_air)

    # 3. Gravity
    g_vec_val = get_gravity_vector(X_pos)

    # 4. Coriolis
    A_coriolis_val = get_coriolis_acceleration(u_ground, lat=launch_latitude, az_launch=launch_azimuth)
    
    # 5. dN/dt (Axial Spin Update - Eq 3.2)
    # Requires: rho, d, A_axial, KA, N, v_air_mag
    dN_dt_val = get_dN_dt(rho_val, d, A_axial, KA, N_val, v_air_mag)
    
    # 6. Acceleration u_dot (Eq 3.1)
    # Requires: rho, C_ballistic (unused), KD_0, KD_a, v_air, d, m, KL, current_ae_vec, g_vec, A_coriolis_val, KF, N_val
    # Q_yaw_drag_factor is calculated inside get_acceleration_vector_u_dot from current_ae_vec
    u_dot_val = get_acceleration_vector_u_dot(
        rho=rho_val, C_ballistic=C_ballistic, KD_drag=KD_0, KD_a_yaw_drag=KD_a,
        v_air_vec=v_air, d_ref=d, m_mass=m, KL_lift=KL, 
        ae_yaw_repose_vec=current_ae_vec, # This is crucial
        g_vec=g_vec_val, A_coriolis_vec=A_coriolis_val, 
        KF_magnus=KF, N_spin=N_val
    )

    # Derivatives: dX/dt = u_ground, du_ground/dt = u_dot_val, dN/dt = dN_dt_val
    derivatives = np.concatenate((u_ground, u_dot_val, np.array([dN_dt_val])))
    return derivatives


# Main Simulation Loop
def run_simulation(X_init, u_init, N_init, ae_init_guess, t_max=300.0, dt_step=0.01, 
                   use_simplified_ae_calc=False, verbose=True):
    """
    Runs the full trajectory simulation.

    Args:
        X_init: Initial position [x,y,z] (m)
        u_init: Initial velocity (ground) [ux,uy,uz] (m/s)
        N_init: Initial axial spin (rad/s)
        ae_init_guess: Initial guess for yaw of repose vector.
        t_max: Maximum simulation time (s).
        dt_step: Time step for integration (s).
        use_simplified_ae_calc: Bool to control ae calculation method.
        verbose: Bool to control print statements.

    Returns:
        tuple: (times, positions, velocities, spins, yaws_of_repose, impact_pos, time_of_flight)
    """
    num_steps = int(t_max / dt_step)
    
    # State vector: [x_pos, y_pos, z_pos, u_ground_x, u_ground_y, u_ground_z, N_spin]
    current_state = np.concatenate((X_init, u_init, np.array([N_init])))
    current_ae = np.copy(ae_init_guess)
    
    time = 0.0
    
    # Store results
    times = [time]
    positions = [np.copy(X_init)]
    velocities_ground = [np.copy(u_init)]
    spins = [N_init]
    yaws_of_repose = [np.copy(current_ae)] # Store ae used for forces at each step

    if verbose:
        print(f"Starting simulation with dt={dt_step}s, t_max={t_max}s")
        print(f"Initial State: X={X_init}, u={u_init}, N={N_init:.2f} rad/s, ae={current_ae}")

    impact_pos_final = None
    time_of_flight = -1.0

    for i in range(num_steps):
        if current_state[2] < 0 and time > dt_step: # Altitude (z = state[2]) < 0, and not at t=0 (allow one step for initial high z)
            # Basic interpolation for impact time and position
            p_prev = positions[-1] # Last stored position
            p_curr = current_state[0:3] # Position that just went negative
            t_prev = times[-1]    # Last stored time
            t_curr = time         # Current time with negative altitude

            if abs(p_curr[2] - p_prev[2]) > 1e-6:
                frac = -p_prev[2] / (p_curr[2] - p_prev[2])
                time_of_flight = t_prev + frac * (t_curr - t_prev)
                impact_pos_final = p_prev + frac * (p_curr - p_prev)
            else: 
                time_of_flight = t_curr
                impact_pos_final = p_curr
            if verbose:
                print(f"Impact detected at t={time_of_flight:.2f} s, position: {impact_pos_final}")
            # Store final impact state before breaking
            times.append(time_of_flight)
            positions.append(np.copy(impact_pos_final))
            # For other states like velocity, spin at impact, one might need to interpolate them too or use last full step's.
            # For simplicity, we'll append the last fully computed state for these if needed later.
            # For now, mainly interested in impact_pos_final and time_of_flight.
            velocities_ground.append(np.copy(current_state[3:6])) # Velocity at last full step before impact step
            spins.append(current_state[6])
            yaws_of_repose.append(np.copy(current_ae))
            break
        
        # Before calling system_derivatives (and thus rk4_step), we need to update ae.
        # ae is solved algebraically based on the *current* state (velocity, spin, etc.)
        # according to Eq (3.3) or simplified form.
        # It uses ae_prev (which is current_ae from the last step) for some terms in full Eq (3.3)
        
        # Get current values from state for ae calculation
        X_pos_curr = current_state[0:3]
        u_ground_curr = current_state[3:6]
        N_curr = current_state[6]
        
        rho_curr = get_air_density(X_pos_curr[2])
        v_air_curr = get_velocity_relative_to_air(u_ground_curr) # Assuming zero wind
        g_vec_curr = get_gravity_vector(X_pos_curr)
        
        # Calculate current ae based on current state and *previous* ae (which is current_ae)
        # The function get_yaw_of_repose_vector_ae takes ae_prev as an argument.
        ae_for_this_step = get_yaw_of_repose_vector_ae(
            v_air_vec=v_air_curr, 
            ae_prev=current_ae, # Use ae from the end of the previous step
            N_spin=N_curr, 
            g_vec=g_vec_curr,
            m_mass=m, d_ref=d, B_trans_inertia=B_transverse, rho=rho_curr,
            KL_lift=KL, KM_pitch_moment=KM, KF_magnus_force=KF, 
            KT_magnus_moment=KT, KN_spin_damping_roll=KN,
            use_simplified_ae=use_simplified_ae_calc
        )
        
        # Now integrate the state [X, u, N] using this ae_for_this_step for force calculations
        # The system_derivatives function will use this fixed ae_for_this_step for the duration of the RK4 sub-steps.
        current_state = rk4_step(system_derivatives, current_state, time, dt_step, current_ae_vec=ae_for_this_step)
        
        # Update current_ae for the *next* iteration's calculation based on the newly calculated ae_for_this_step
        current_ae = ae_for_this_step 
        
        time += dt_step
        
        # Store results for this step
        times.append(time)
        positions.append(np.copy(current_state[0:3]))
        velocities_ground.append(np.copy(current_state[3:6]))
        spins.append(current_state[6])
        yaws_of_repose.append(np.copy(current_ae)) # Store the ae that was used for this step's forces

        if verbose and (i + 1) % (int(1.0 / dt_step) * 10) == 0: # Print progress e.g. every 10 seconds of sim time
             print(f"t={time:.2f}s, X=[{current_state[0]:.1f}, {current_state[1]:.1f}, {current_state[2]:.1f}]m, V_mag={np.linalg.norm(current_state[3:6]):.1f} m/s, N={current_state[6]:.1f} rad/s, ae_norm={np.rad2deg(np.linalg.norm(current_ae)):.2f} deg")
    else: # Loop finished without break (no impact within t_max)
        if verbose:
            print(f"Simulation completed up to t_max = {t_max:.2f} s without impact.")
            print(f"Final State: X={current_state[0:3]}, u={current_state[3:6]}, N={current_state[6]:.2f} rad/s, ae={current_ae}")

    # Return all trajectory data, plus specific impact data
    return times, positions, velocities_ground, spins, yaws_of_repose, impact_pos_final, time_of_flight

# Helper to get simulated range for a given elevation
def get_simulated_range(elevation_deg, muzzle_velocity, launch_azimuth_deg, N_init_val, ae_init_val,
                        sim_t_max, sim_dt_step, use_simplified_ae, verbose_sim=False):
    """
    Runs a simulation for a given elevation and muzzle velocity, and returns the impact range (x-coordinate).
    Returns -1.0 if no impact or invalid impact.
    """
    u0_elevation_rad = np.deg2rad(elevation_deg)
    u0_azimuth_sim_rad = np.deg2rad(launch_azimuth_deg) # Azimuth of velocity vector in sim frame

    sim_u_init = np.array([
        muzzle_velocity * np.cos(u0_elevation_rad) * np.cos(u0_azimuth_sim_rad),
        muzzle_velocity * np.cos(u0_elevation_rad) * np.sin(u0_azimuth_sim_rad),
        muzzle_velocity * np.sin(u0_elevation_rad)
    ])
    # Start slightly above ground to avoid immediate impact issues with z=0
    sim_X_init = np.array([0.0, 0.0, 0.1]) 

    _, _, _, _, _, impact_position, _ = run_simulation(
        X_init=sim_X_init, 
        u_init=sim_u_init, 
        N_init=N_init_val, 
        ae_init_guess=ae_init_val,
        t_max=sim_t_max, 
        dt_step=sim_dt_step, 
        use_simplified_ae_calc=use_simplified_ae,
        verbose=verbose_sim
    )

    if impact_position is not None and impact_position[0] > 0: # Ensure forward range
        return impact_position[0] # Return X-component of impact
    else:
        return -1.0 # Indicate no valid impact / range


# Bisection search to find elevation for a target range
def find_elevation_for_range(target_range_m, muzzle_velocity_mps, 
                             N_initial, ae_initial, 
                             sim_t_max_secs, sim_dt_s, use_simp_ae,
                             launch_az_deg=0.0, # Assuming firing along x-axis for range calculation
                             min_elev_deg=0.1, max_elev_deg=60.0, # Search primarily lower trajectories
                             tolerance_m=10.0, max_iterations=30):
    """
    Finds the launch elevation angle (degrees) to achieve a target range.
    Uses the bisection method.
    
    Args:
        target_range_m: Desired impact range (x-coordinate) in meters.
        muzzle_velocity_mps: Muzzle velocity in m/s.
        N_initial, ae_initial: Initial spin and yaw of repose.
        sim_t_max_secs, sim_dt_s, use_simp_ae: Simulation control parameters.
        launch_az_deg: Launch azimuth for the simulation (usually 0 for simple range finding).
        min_elev_deg, max_elev_deg: Search interval for elevation.
        tolerance_m: Acceptable error in range (meters).
        max_iterations: Maximum iterations for the bisection search.
        
    Returns:
        float: Required elevation angle in degrees, or None if not found.
    """
    print(f"\nAttempting to find elevation for target range: {target_range_m:.1f} m with muzzle velocity: {muzzle_velocity_mps:.1f} m/s")

    # --- Pre-check: Estimate maximum achievable range with this velocity --- 
    print("  Estimating maximum achievable range for this muzzle velocity and projectile model...")
    estimated_max_range = -1.0
    elev_for_estimated_max_range = -1.0
    # Coarse search for max range, typically between 30-50 deg with air resistance.
    # We'll scan a slightly wider practical range for safety.
    # Test specific angles known to be around optimal, plus the bounds of the user's typical search.
    potential_optimal_elevs = np.linspace(max(min_elev_deg, 25.0), min(max_elev_deg, 55.0), 7) # e.g. 25, 30, 35, 40, 45, 50, 55 if bounds allow
    
    # Initial ranges at bounds (already calculated if we decide to keep original low/high_elev evaluation first)
    # For max range estimation, we re-evaluate to be self-contained here.
    current_low_elev_range = get_simulated_range(min_elev_deg, muzzle_velocity_mps, launch_az_deg,
                                             N_initial, ae_initial, sim_t_max_secs, sim_dt_s, use_simp_ae, verbose_sim=False)
    current_high_elev_range = get_simulated_range(max_elev_deg, muzzle_velocity_mps, launch_az_deg,
                                             N_initial, ae_initial, sim_t_max_secs, sim_dt_s, use_simp_ae, verbose_sim=False)

    if current_low_elev_range > estimated_max_range:
        estimated_max_range = current_low_elev_range
        elev_for_estimated_max_range = min_elev_deg
    if current_high_elev_range > estimated_max_range:
        estimated_max_range = current_high_elev_range
        elev_for_estimated_max_range = max_elev_deg

    for test_elev in potential_optimal_elevs:
        r = get_simulated_range(test_elev, muzzle_velocity_mps, launch_az_deg,
                                N_initial, ae_initial, sim_t_max_secs, sim_dt_s, use_simp_ae, verbose_sim=False)
        if r > estimated_max_range:
            estimated_max_range = r
            elev_for_estimated_max_range = test_elev
    
    print(f"  Estimated max range for this velocity & model: ~{estimated_max_range:.1f} m at ~{elev_for_estimated_max_range:.2f} deg / {elev_for_estimated_max_range*DEG_TO_MILS:.1f} mils")

    if target_range_m > estimated_max_range + tolerance_m: # Add tolerance to comparison
        print(f"  Target range {target_range_m:.1f} m is GREATER than the estimated maximum achievable range ({estimated_max_range:.1f} m).")
        print("  Cannot find a solution for this target range with the current projectile model and muzzle velocity.")
        return None
    # --- End of Pre-check --- 

    low_elev = min_elev_deg
    high_elev = max_elev_deg

    # Evaluate initial bounds for bisection (can reuse from above if logic is aligned)
    range_at_low_elev = current_low_elev_range # Re-use from max range check
    # print(f"  Range at min_elev ({low_elev:.2f} deg / {low_elev * DEG_TO_MILS:.1f} mils): {range_at_low_elev:.1f} m") # Already printed by max range if verbose
    
    range_at_high_elev = current_high_elev_range # Re-use from max range check
    # print(f"  Range at max_elev ({high_elev:.2f} deg / {high_elev * DEG_TO_MILS:.1f} mils): {range_at_high_elev:.1f} m")

    # Function for bisection: f(elev) = simulated_range(elev) - target_range
    f_low = range_at_low_elev - target_range_m
    f_high = range_at_high_elev - target_range_m

    # Initial bracketing check for bisection
    if (range_at_low_elev < 0 and range_at_high_elev < 0):
        print("  Both initial elevation bounds resulted in no impact or invalid range. Cannot reliably proceed with bisection.")
        return None

    if f_low * f_high > 0 and (range_at_low_elev > 0 and range_at_high_elev > 0): # Both have same sign AND valid ranges
        print(f"  Warning: Target range {target_range_m:.1f}m is not strictly bracketed by ranges at initial elevation bounds.")
        print(f"           ({low_elev:.2f}deg: {range_at_low_elev:.1f}m vs {high_elev:.2f}deg: {range_at_high_elev:.1f}m). Expecting one range < target and one > target.")
        print(f"           f_low = {f_low:.1f}, f_high = {f_high:.1f}")
        print("           Bisection might fail or find an unexpected solution. This can happen if target is near max range or if search bounds are wide.")
        # Allow proceeding, as the target might be between one bound and a peak within the interval.

    # Check if target is already met at the boundaries (more robustly)
    if abs(f_low) < tolerance_m and range_at_low_elev > 0:
        print(f"Target range ~{target_range_m:.1f}m achieved at min_elev boundary {low_elev:.2f} deg / {low_elev * DEG_TO_MILS:.1f} mils (Range: {range_at_low_elev:.1f}m).")
        return low_elev
    if abs(f_high) < tolerance_m and range_at_high_elev > 0:
        print(f"Target range ~{target_range_m:.1f}m achieved at max_elev boundary {high_elev:.2f} deg / {high_elev * DEG_TO_MILS:.1f} mils (Range: {range_at_high_elev:.1f}m).")
        return high_elev

    # The old bracketing message based on relative order - can be removed or kept if desired
    # if not ((range_at_low_elev < target_range_m < range_at_high_elev) or 
    #         (range_at_high_elev < target_range_m < range_at_low_elev)):
    #     print("Target range may not be bracketed by ranges at initial elevation bounds.")

    for iteration in range(max_iterations):
        mid_elev = (low_elev + high_elev) / 2.0
        current_range = get_simulated_range(mid_elev, muzzle_velocity_mps, launch_az_deg, 
                                            N_initial, ae_initial, sim_t_max_secs, sim_dt_s, use_simp_ae, verbose_sim=False)
        
        if current_range == -1.0: # Projectile didn't hit, e.g., flew too high or too short within t_max
            # This is tricky for bisection. Assume it means we need to adjust elevation.
            # If current_range is -1, it means it probably fell short or went too vertical.
            # If we assume range generally increases with elevation up to a point, then if mid is -1,
            # and high_elev gave a positive range, try lower half. 
            # This logic might need refinement based on how -1.0 is returned (undershoot vs over-vertical)
            # For now, if mid is -1, let's assume it undershot or went too vertical (similar to short range)
            # This means we might need a higher elevation if target_range is positive.
            # Or a lower elevation if it went too vertical. 
            # This makes simple bisection hard if -1 can mean different things.
            # Let's assume -1.0 means it fell short / didn't reach target plane in reasonable way.
            print(f"Iter {iteration+1}: elev={mid_elev:.3f} deg / {mid_elev * DEG_TO_MILS:.1f} mils, range=NO_IMPACT. Adjusting search.")
            # If range_at_high_elev was positive and greater than target, then no_impact at mid suggests mid was too low or too high (vertical).
            # This logic is fragile. A more robust solver (e.g. Brent's method) might be better.
            # Simple bisection: if we assume range increases with elevation (up to 45 deg), no impact means too low an angle OR too high (straight up)
            # Let's assume for search_elev < 45-ish, no impact means too low.
            if mid_elev < 40 : # Guess that no impact for low angles means angle too low
                 low_elev = mid_elev
            else: # Guess that no impact for higher angles means angle too high (too vertical)
                 high_elev = mid_elev
            continue # Try next iteration

        error = current_range - target_range_m
        print(f"Iter {iteration+1}: elev={mid_elev:.3f} deg / {mid_elev * DEG_TO_MILS:.1f} mils, range={current_range:.1f} m, error={error:.1f} m")

        if abs(error) < tolerance_m:
            print(f"Solution found: Elevation = {mid_elev:.3f} degrees / {mid_elev * DEG_TO_MILS:.1f} mils for range {current_range:.1f} m.")
            return mid_elev

        # Determine which half to keep. This depends on the relationship between elevation and range.
        # For elevations < ~45 deg (lower trajectory), range typically increases with elevation.
        # For elevations > ~45 deg (higher trajectory), range typically decreases with elevation.
        # Our search is typically for the lower trajectory (min_elev_deg to max_elev_deg usually < 60).
        
        # Assuming range increases with elevation in the search interval [min_elev_deg, max_elev_deg]
        # This is true for the lower firing solution (typically < 45 deg).
        # If current_range < target_range_m, we need more elevation.
        # If current_range > target_range_m, we need less elevation.
        # This logic also needs f(low_elev) and f(high_elev) to have opposite signs relative to target.

        # Re-evaluate f(low_elev) and f(high_elev) for robustness if one was -1.
        err_low_check = get_simulated_range(low_elev, muzzle_velocity_mps, launch_az_deg, 
                                             N_initial, ae_initial, sim_t_max_secs, sim_dt_s, use_simp_ae, verbose_sim=False) - target_range_m
        # err_mid_check = current_range - target_range_m (already have this as error)

        if err_low_check * error < 0: # Root is between low_elev and mid_elev
            high_elev = mid_elev
        else: # Root is between mid_elev and high_elev (or low_elev was the one to move)
            low_elev = mid_elev
            # Update range_at_low_elev to current_range if low_elev is moved to mid_elev
            # This helps if initial range_at_low_elev was -1.
            # No, this is not right. The error_low_check should be sufficient.

    print("Bisection search did not converge to the target range within tolerance.")
    return None


# placeholder for main simulation
if __name__ == "__main__":
    print("--- Artillery Range Solver ---")

    # Get Initial Conditions from User Input
    while True:
        try:
            u0_mag_str = input("Enter muzzle velocity (m/s) [e.g., 750.0]: ")
            u0_mag = float(u0_mag_str)
            if u0_mag <= 0:
                print("Muzzle velocity must be positive.")
                continue
            break
        except ValueError:
            print("Invalid input. Please enter a number for muzzle velocity.")

    while True:
        try:
            target_range_str = input("Enter desired target range (meters) [e.g., 10000.0]: ")
            target_range_val = float(target_range_str)
            if target_range_val <= 0:
                print("Target range must be positive.")
                continue
            break
        except ValueError:
            print("Invalid input. Please enter a number for target range.")

    # Sim params for solver and final run (can be adjusted)
    sim_t_max = 300.0 
    sim_dt = 0.05
    # Set use_simplified_ae_calc globally for the solver and final run, or pass as arg
    use_simplified_ae_for_solver = False 
    # Global projectile params N0, ae0 are used for N_initial, ae_initial
    # Global launch_azimuth (degrees) for sim frame
    launch_azimuth_for_sim_deg = 0.0 # Fire along X-axis for range calculation

    # Find the elevation
    found_elevation_deg = find_elevation_for_range(
        target_range_m=target_range_val, 
        muzzle_velocity_mps=u0_mag,
        N_initial=N0, 
        ae_initial=ae0, 
        sim_t_max_secs=sim_t_max, 
sim_dt_s=sim_dt, 
        use_simp_ae=use_simplified_ae_for_solver,
        launch_az_deg=launch_azimuth_for_sim_deg,
        min_elev_deg=1.0,  # Start search from low angle
        max_elev_deg=55.0, # Max practical lower trajectory angle
        tolerance_m=20.0,  # Tolerance in meters
        max_iterations=25
    )

    if found_elevation_deg is not None:
        print(f"\n--- Running Final Simulation with Found Elevation: {found_elevation_deg:.3f} degrees / {found_elevation_deg * DEG_TO_MILS:.1f} mils ---")
        
        u0_elevation_rad = np.deg2rad(found_elevation_deg)
        u0_az_sim_rad = np.deg2rad(launch_azimuth_for_sim_deg) # Using the same azimuth

        sim_u_init = np.array([
            u0_mag * np.cos(u0_elevation_rad) * np.cos(u0_az_sim_rad),
            u0_mag * np.cos(u0_elevation_rad) * np.sin(u0_az_sim_rad),
            u0_mag * np.sin(u0_elevation_rad)
        ])
        sim_X_init = np.array([0.0, 0.0, 0.1])
        sim_N_init = N0 
        sim_ae_init = ae0

        # Import Matplotlib for plotting (ensure it's available for the final run)
        try:
            import matplotlib.pyplot as plt
            plotting_available = True
        except ImportError:
            print("Matplotlib not found. Plotting will be skipped for the final run.")
            plotting_available = False

        sim_times, sim_positions, sim_velocities, sim_spins, sim_aes, impact_pos, impact_time = run_simulation(
            X_init=sim_X_init, 
            u_init=sim_u_init, 
            N_init=sim_N_init, 
            ae_init_guess=sim_ae_init,
            t_max=sim_t_max, 
            dt_step=sim_dt, 
            use_simplified_ae_calc=use_simplified_ae_for_solver, 
            verbose=True
        )

        if plotting_available and sim_times:
            final_pos = sim_positions[-1]
            # ... (rest of the plotting code, slightly adapted if needed) ...
            positions_arr = np.array(sim_positions)
            times_arr = np.array(sim_times)
            # ... (ensure all plot variables are defined) ...
            # (The existing plotting code should largely work if sim_times, etc. are populated)
            fig, axs = plt.subplots(3, 2, figsize=(15, 18))
            fig.suptitle(f"Artillery Sim - Target Range: {target_range_val:.0f}m, Muzzle Vel: {u0_mag} m/s, Launch Elev: {found_elevation_deg:.2f} deg / {found_elevation_deg * DEG_TO_MILS:.1f} mils", fontsize=14)

            # 1. Trajectory (Side View: Z vs X)
            ax = axs[0, 0]
            ax.plot(positions_arr[:,0], positions_arr[:,2], label="Trajectory")
            if impact_pos is not None:
                ax.scatter([impact_pos[0]], [impact_pos[2]], color='red', zorder=5, label=f"Impact ({impact_pos[0]:.1f}m)")
            ax.set_xlabel("Range (X, m)")
            ax.set_ylabel("Altitude (Z, m)")
            ax.set_title("Projectile Trajectory (Side View)")
            ax.grid(True)
            ax.axhline(0, color='black', linewidth=0.5)
            ax.legend()

            # 2. Ground Track (Y vs X)
            ax = axs[0, 1]
            ax.plot(positions_arr[:,0], positions_arr[:,1], label="Ground Track")
            if impact_pos is not None:
                ax.scatter([impact_pos[0]], [impact_pos[1]], color='red', zorder=5, label=f"Impact ({impact_pos[1]:.1f}m defl.)")
            ax.set_xlabel("Range (X, m)")
            ax.set_ylabel("Deflection (Y, m)")
            ax.set_title("Projectile Ground Track")
            ax.grid(True)
            ax.axhline(0, color='black', linewidth=0.5)
            ax.axvline(0, color='black', linewidth=0.5)
            ax.axis('equal')
            ax.legend()

            # 3. Altitude vs Time
            ax = axs[1, 0]
            velocities_arr = np.array(sim_velocities) # Ensure this is defined before use if not already
            spins_arr = np.array(sim_spins)
            aes_arr = np.array(sim_aes)
            ax.plot(times_arr, positions_arr[:,2], label="Altitude")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Altitude (Z, m)")
            ax.set_title("Altitude vs. Time")
            ax.grid(True)
            if impact_time > 0:
                ax.axvline(impact_time, color='red', linestyle='--', label=f"Impact ({impact_time:.1f}s)")
            ax.legend()

            # 4. Speed vs Time
            ax = axs[1, 1]
            speeds_mag = np.linalg.norm(velocities_arr, axis=1)
            ax.plot(times_arr, speeds_mag, label="Speed (Ground)")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Speed (m/s)")
            ax.set_title("Projectile Speed vs. Time")
            ax.grid(True)
            if impact_time > 0:
                ax.axvline(impact_time, color='red', linestyle='--', label=f"Impact ({impact_time:.1f}s)")
            ax.legend()
            
            # 5. Spin Rate (N) vs Time
            ax = axs[2, 0]
            ax.plot(times_arr, spins_arr * (1/(2*np.pi)), label="Spin Rate (rev/s)")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Axial Spin Rate (rev/s)")
            ax.set_title("Axial Spin Rate vs. Time")
            ax.grid(True)
            if impact_time > 0:
                ax.axvline(impact_time, color='red', linestyle='--', label=f"Impact ({impact_time:.1f}s)")
            ax.legend()

            # 6. Yaw of Repose (ae) Magnitude vs Time
            ax = axs[2, 1]
            aes_mag = np.linalg.norm(aes_arr, axis=1)
            ax.plot(times_arr, np.rad2deg(aes_mag), label="|ae| (degrees)")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("|ae| (degrees)")
            ax.set_title("Yaw of Repose Magnitude (|ae|) vs. Time (degrees)")
            ax.grid(True)
            if impact_time > 0:
                ax.axvline(impact_time, color='red', linestyle='--', label=f"Impact ({impact_time:.1f}s)")
            ax.legend()

            fig.subplots_adjust(hspace=0.4, wspace=0.3)
            plt.tight_layout(rect=[0, 0, 1, 0.95]) # Adjusted rect for potentially longer title
            plt.show()
        elif not sim_times:
            print("Final simulation did not produce data for plotting.")

    else:
        print("\nCould not find a suitable elevation angle for the given target range and muzzle velocity.")
        print("Try adjusting muzzle velocity, target range, or the elevation search interval/parameters.")


    # Old plotting section (now integrated into the above conditional block)
    # if plotting_available:
    # ...