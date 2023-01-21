import math

def RadToDeg(rad):
    return rad * 180.0 / math.pi

def QuatToEuler(quat):
    qx, qy, qz, qw = quat
    sinR_cosP = 2.0 * (qw * qx + qy * qz)
    cosR_cosP = 1.0 - 2.0 * (qx ** 2.0 + qy ** 2.0)
    roll = math.atan2(sinR_cosP, cosR_cosP)
    sinP = math.sqrt(1.0 + 2.0 * (qw * qy + qy * qz))
    cosP = math.sqrt(1.0 - 2.0 * (qw * qy - qx * qz))
    pitch = 2.0 * math.atan2(sinP, cosP) - math.pi / 2.0
    sinY_copP = 2.0 * (qw * qz + qx * qy)
    cosY_cosP = 1.0 - 2.0 * (qy ** 2.0 + qz ** 2.0)
    yaw = math.atan2(sinY_copP, cosY_cosP)
    return list(map(RadToDeg, [roll, pitch, yaw]))


def normalizeQuat(quat):
    qx, qy, qz, qw = quat
    norm = float(math.sqrt(qw ** 2.0 + qx ** 2.0 + qy ** 2.0 + qz ** 2.0))
    return list(map(lambda x: float(x / norm), quat))


def invQuat(quat):
    qx, qy, qz, qw = quat
    return [-qx, -qy, -qz, qw]


def EulerToQuat(euler):
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


def MultiplicationQuat(quat1, quat2):
    b1, c1, d1, a1 = quat1
    b2, c2, d2, a2 = quat2
    qw = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2
    qx = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2
    qy = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2
    qz = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2
    return normalizeQuat([qx, qy, qz, qw])


def getChangedOrientation(quat_now, quat_last):
    quat = MultiplicationQuat(quat_now, invQuat(quat_last))
    return normalizeQuat(quat)
