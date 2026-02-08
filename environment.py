# --- Environment ---
G_earth = 9.81  # m/s^2
r_moon = 1737e3  # m
G = 6.67408e-11  # m^3/kg/s^2
m_moon = 7.34767309e22  # kg
mu = G * m_moon  # m^3/s^2

# --- Vehicle Constants ---
T_max = 45_000  # N
Isp = 311  # s
m_empty = 7_201  # kg
m_prop0 = 8_134  # kg
m0 = m_prop0 + m_empty  # kg
alpha0 = 0  # rad
dalpha_max = 0.10472  # rad/s
