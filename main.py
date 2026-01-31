import numpy as np

G0 = 1.625  # m/s^2

dt = 1  # s


class Lander:
    def __init__(self, S, Isp, T, m_e):
        self.S = S
        self.Isp = Isp
        self.T = T
        self.m_e = m_e
        self.m_p = S[2] - m_e

    def find_acceleration(self, S, T, G0):
        a = (T / S[2]) - G0

        return a

    def find_velocity(self, S, a, dt):
        v = S[1] + a * dt

        S[1] = v

        return v

    def find_position(self, S, v, dt):
        r = S[0] + v * dt

        S[0] = r

        return r

    def update_mass(self, S, T, Isp, G0, m_e):
        dmdt = T / (Isp * G0)

        if S[2] - dmdt * dt >= m_e:
            S[2] -= dmdt * dt


Apollo = Lander(np.array([10000, -100, 15200]), 311, 45000 * 0.2, 4280)


for dt in range(1, 101):
    print(Apollo.S)

    a = Apollo.find_acceleration(Apollo.S, Apollo.T, G0)

    v = Apollo.find_velocity(Apollo.S, a, dt)

    Apollo.find_position(Apollo.S, v, dt)

    Apollo.update_mass(Apollo.S, Apollo.T, Apollo.Isp, G0, Apollo.m_e)
