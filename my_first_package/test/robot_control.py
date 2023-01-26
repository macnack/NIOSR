import math


def rad_to_deg(rad):
    return rad * 180.0 / math.pi


def quat_to_euler(quat):
    qx, qy, qz, qw = quat
    pitch = 0
    sinR_cosP = 2.0 * (qw * qx + qy * qz)
    cosR_cosP = 1.0 - 2.0 * (qx ** 2.0 + qy ** 2.0)
    roll = math.atan2(sinR_cosP, cosR_cosP)
    try:
        sinP = math.sqrt(1.0 + 2.0 * (qw * qy + qy * qz))
        cosP = math.sqrt(1.0 - 2.0 * (qw * qy - qx * qz))
        pitch = 2.0 * math.atan2(sinP, cosP) - math.pi / 2.0
    except ValueError as e:
        print(e)
    sinY_copP = 2.0 * (qw * qz + qx * qy)
    cosY_cosP = 1.0 - 2.0 * (qy ** 2.0 + qz ** 2.0)
    yaw = math.atan2(sinY_copP, cosY_cosP)
    return list(map(rad_to_deg, [roll, pitch, yaw]))


def normalize_quat(quat):
    qx, qy, qz, qw = quat
    norm = float(math.sqrt(qw ** 2.0 + qx ** 2.0 + qy ** 2.0 + qz ** 2.0))
    return list(map(lambda x: float(x / norm), quat))


def inv_quat(quat):
    qx, qy, qz, qw = quat
    return [-qx, -qy, -qz, qw]


def euler_to_quat(euler):
    roll, pitch, yaw = euler
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return [qx, qy, qz, qw]


def multiplication_quat(quat1, quat2):
    b1, c1, d1, a1 = quat1
    b2, c2, d2, a2 = quat2
    qw = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2
    qx = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2
    qy = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2
    qz = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2
    return normalize_quat([qx, qy, qz, qw])


def get_changed_orientation(quat_now, quat_last):
    quat = multiplication_quat(quat_now, inv_quat(quat_last))
    return normalize_quat(quat)
