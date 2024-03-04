"""
Set of utility functions used across the package.
"""

import casadi as ca


def r_mat(euler):
    # Extract angles
    phi = euler[0]
    theta = euler[1]
    psi = euler[2]

    # Calculate cos/sin
    cp, sp = ca.cos(phi), ca.sin(phi)
    ct, st = ca.cos(theta), ca.sin(theta)
    cs, ss = ca.cos(psi), ca.sin(psi)

    Rmat = ca.MX.zeros(3, 3)

    Rmat[0, 0] = cs * ct
    Rmat[0, 1] = cs * st * sp - ss * cp
    Rmat[0, 2] = cs * st * cp + ss * sp

    Rmat[1, 0] = ss * ct
    Rmat[1, 1] = ss * st * sp + cs * cp
    Rmat[1, 2] = ss * st * cp - cs * sp

    Rmat[2, 0] = -st
    Rmat[2, 1] = ct * sp
    Rmat[2, 2] = ct * cp

    return Rmat


def rot_jac_mat(euler):

    phi = euler[0]
    theta = euler[1]

    H = ca.MX.zeros((3, 3))
    H[0, 0] = 1
    H[0, 1] = ca.sin(phi) * ca.tan(theta)
    H[0, 2] = ca.cos(phi) * ca.tan(theta)

    H[1, 1] = ca.cos(phi)
    H[1, 2] = -ca.sin(phi)

    H[2, 1] = ca.sin(phi) / ca.cos(theta)
    H[2, 2] = ca.cos(phi) / ca.cos(theta)

    return H


def skew(v):
    """
    Returns the skew matrix of a vector v

    :param v: vector
    :type v: ca.MX
    :return: skew matrix of v
    :rtype: ca.MX
    """

    sk = ca.MX.zeros(3, 3)

    # Extract vector components
    x = v[0]
    y = v[1]
    z = v[2]

    sk[0, 1] = -z
    sk[1, 0] = z
    sk[0, 2] = y
    sk[2, 0] = -y
    sk[1, 2] = -x
    sk[2, 1] = x

    return sk