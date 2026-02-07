import numpy as np
import matplotlib.pyplot as plt
import simulation as sim
import guidance as gd
import navigation as nav
import environment as env

# --- Constants & Environment ---
r_moon = env.r_moon
mu = env.mu

S0 = [
    14_878 + r_moon,
    0,
    0,
    np.sqrt(mu / (r_moon + 14_878)) / (r_moon + 14_878),
    15_240,
]

t_vec = np.linspace(0, 500, 500)

LVLH = nav.polar_to_LVLH(S0)
alt = nav.altitude(LVLH)
z_cmd, _, _ = gd.poly_guidance(t_vec, [LVLH[0], 0, LVLH[1], 0], 500)
x_cmd, _, _ = gd.poly_guidance(t_vec, [LVLH[2], 480_000, LVLH[3], 0], 500)

plt.plot(x_cmd, -z_cmd)
plt.xlabel("x (m)")
plt.ylabel("altitude (m)")
plt.title("Guidance Trajectory")
plt.grid(True)
plt.show()
