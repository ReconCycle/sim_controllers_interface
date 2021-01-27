#!/usr/bin/env python3
import numpy as np


def min_jerk_slerp(q1, q2, T, t):
    '''
    MIN_JERK_SLERP: spherical linear interpolation from quaternion q1 to
                    quaternion q2 in time T. The evolution of time follows
                    the minimum jerk trajectory with zero velocities and
                    accelerations at end-points.

    See Ken Shoemake, Animating Rotation with Quaternion Curves, SIGGRAPH 1985.

    INPUTS:
        q1: initial quaternion orientation
        q2: final quaternion orientation
        T: duration of movement (> 0)
        t: current time (0 <= t <= T)

    OUTPUTS:
        q: quaternion orientation at time t
        omega: angular velocity at time t
    '''
    if T <= 0:
        q = q1.copy()
        omega = np.asarray([0.0, 0.0, 0.0])
        alpha = np.asarray([0.0, 0.0, 0.0])
        return (q, omega, alpha)

    dq = quat_mult(q2, quat_conjugate(q1))
    dq_1 = np.sqrt(dq['s']**2 + dq['v'][0]**2 + dq['v'][1]**2 + dq['v'][2]**2)
    dq['s'] = dq['s'] / dq_1
    dq['v'] = dq['v'] / dq_1
    th = np.arccos(dq['s'])

    if th > np.pi/2:
        # wrong direction, path towards -q2 (same orientation) is shorter
        q2['s'] = -q2['s']
        q2['v'] = -q2['v']

        dq = quat_mult(q2, quat_conjugate(q1))
        th = np.arccos(dq['s'])

    a = minimum_jerk_polynomial(T)
    sth = np.sin(th)

    if t <= 0:
        q = q1.copy()
        omega = np.asarray([0.0, 0.0, 0.0])
        alpha = np.asarray([0.0, 0.0, 0.0])
    elif t >= T or sth < np.spacing(1):
        q = q2.copy()
        omega = np.asarray([0.0, 0.0, 0.0])
        alpha = np.asarray([0.0, 0.0, 0.0])
    else:
        t, dt, ddt = minimum_jerk(t, a)

        sth1 = np.sin((1-t)*th) / sth
        sth2 = np.sin(t*th) / sth

        q = dict()
        q['s'] = sth1 * q1['s'] + sth2 * q2['s']
        q['v'] = sth1 * q1['v'] + sth2 * q2['v']

        cth1 = np.cos((1-t)*th) / sth
        cth2 = np.cos(t*th) / sth

        scth1 = -sth1 * (dt*th) * (dt*th) - cth1 * (ddt*th)
        scth2 = -sth2 * (dt*th) * (dt*th) + cth2 * (ddt*th)

        cth1 = -cth1 * (dt*th)
        cth2 = cth2 * (dt*th)

        dq['s'] = cth1 * q1['s'] + cth2 * q2['s']
        dq['v'] = cth1 * q1['v'] + cth2 * q2['v']

        ddq = dict()
        ddq['s'] = scth1 * q1['s'] + scth2 * q2['s']
        ddq['v'] = scth1 * q1['v'] + scth2 * q2['v']

        omega = quat_mult(dq, quat_conjugate(q))
        omega = 2.0 * omega['v']

        alpha = quat_mult(ddq, quat_conjugate(q))
        alpha = 2.0 * alpha['v']

    return (q, omega, alpha)


def quat_mult(q1, q2):
    '''Quaternion multiplication'''
    q = dict()
    q['s'] = q1['s'] * q2['s'] - q1['v'][0] * q2['v'][0] - q1['v'][1] * q2['v'][1] - q1['v'][2] * q2['v'][2]
    q['v'] = q1['s'] * q2['v'] + q2['s'] * q1['v']
    q['v'][0] = q['v'][0] + q1['v'][1]*q2['v'][2] - q1['v'][2]*q2['v'][1]
    q['v'][1] = q['v'][1] + q1['v'][2]*q2['v'][0] - q1['v'][0]*q2['v'][2]
    q['v'][2] = q['v'][2] + q1['v'][0]*q2['v'][1] - q1['v'][1]*q2['v'][0]

    return q


def quat_conjugate(q):
    '''Quaternion conjugation'''
    qc = dict()
    qc['s'] = q['s']
    qc['v'] = -q['v']

    return qc


def minimum_jerk_polynomial(T):
    '''
    Coefficients of minimum jerk polynomial from 0 to 1 in time T with zero
    velocities and accelerations at initial (0) and final (1) point
    '''
    a = np.zeros(6)
    t = T * T * T
    a[3] = 10.0 / t
    t = t * T
    a[4] = -15.0 / t
    t = t * T
    a[5] = 6.0 / t

    return a


def minimum_jerk(t, a):
    '''
    Value, velocity, and acceleration of minimum jerk polynomial at time t
    '''
    t2 = t * t
    t3 = t2 * t
    t4 = t2 * t2
    t5 = t3 * t2
    pos = a[5] * t5 + a[4] * t4 + a[3] * t3
    vel = 5 * a[5] * t4 + 4 * a[4] * t3 + 3 * a[3] * t2
    acc = 20 * a[5] * t3 + 12 * a[4] * t2 + 6 * a[3] * t

    return (pos, vel, acc)