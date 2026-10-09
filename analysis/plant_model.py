"""Linearised reaction-arm pendulum straight from a URDF, so the model and the sim share one source.

State x = [theta_base, omega_base, omega_arm (relative, as the joint reports it)], input u =
torque on body_arm. With I_p = everything's inertia about the pivot except the arm's own spin,
and I_a = the arm's spin inertia:
    theta_ddot = (mgl * theta - u) / I_p
    omega_arm_dot = -theta_ddot + u / I_a
Reproduces analysis/lqr_design.py's hand-derived A, B for urdf/body.urdf (40.9, 556, 53187).

    python3 analysis/plant_model.py urdf/mrv.urdf
"""

import sys
import xml.etree.ElementTree as ET

import numpy as np

G = 9.81


def _floats(s):
    return [float(v) for v in s.split()]


def load(path):
    """Return (A, B, info) for a URDF with joints base_body (pivot) and body_arm, both about y."""
    root = ET.parse(path).getroot()
    links = {l.get('name'): l for l in root.findall('link')}
    joints = {j.get('name'): j for j in root.findall('joint')}
    arm_origin = _floats(joints['body_arm'].find('origin').get('xyz'))

    def inertial(name, offset):
        i = links[name].find('inertial')
        m = float(i.find('mass').get('value'))
        x, _, z = (a + b for a, b in zip(_floats(i.find('origin').get('xyz')), offset))
        return m, x, z, float(i.find('inertia').get('iyy'))

    mb, xb, zb, iyy_b = inertial('body', (0, 0, 0))
    ma, xa, za, i_a = inertial('arm', arm_origin)
    mgl = G * (mb * zb + ma * za)
    i_p = iyy_b + mb * (xb ** 2 + zb ** 2) + ma * (xa ** 2 + za ** 2)  # arm as a point mass
    a2 = mgl / i_p
    A = np.array([[0.0, 1.0, 0.0], [a2, 0.0, 0.0], [-a2, 0.0, 0.0]])
    B = np.array([0.0, -1.0 / i_p, 1.0 / i_p + 1.0 / i_a])
    info = dict(mass=mb + ma, com=(mb * zb + ma * za) / (mb + ma), mgl=mgl, I_p=i_p, I_a=i_a,
                pole=np.sqrt(a2))
    return A, B, info


if __name__ == '__main__':
    A, B, info = load(sys.argv[1])
    print({k: float(f'{v:.4g}') for k, v in info.items()})
    print('A =', A.round(2).tolist())
    print('B =', B.round(1).tolist())
