# --- Lunar Environment Constants ---
G_earth = 9.81  # Standard gravity on Earth (m/s^2) for Isp calculations
r_moon = 1737e3  # Mean radius of the Moon (m)
G = 6.67408e-11  # Universal gravitational constant (m^3/kg/s^2)
m_moon = 7.34767309e22  # Mass of the Moon (kg)
mu = G * m_moon  # Standard gravitational parameter for the Moon (m^3/s^2)

# --- Lunar Module (LM) Vehicle Specifications ---
T_max = 45_000  # Maximum Descent Propulsion System (DPS) thrust (N)
Isp = 311  # Specific impulse of the DPS engine (s)
m_empty = 7_201  # Dry mass of the vehicle (kg)
m_prop0 = 8_134  # Initial propellant mass (kg)
m0 = m_prop0 + m_empty  # Total initial wet mass (kg)
alpha0 = 0  # Initial pitch angle (rad)
dalpha_max = 0.10472  # Maximum gimbal/slew rate (rad/s) (~6 deg/s)
