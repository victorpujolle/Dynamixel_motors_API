import numpy as np

def pseudo_conversion(n, length_res=2):
    """
    :param n: integer
    :param length_res: the length of the message you want, if it is two, the list will have two elements
    :return: the decomposition of n in the base 256, the strongest bit the later
    """
    if n == 0:
        return [0,0]

    res = []
    while n>256:
        k = n%256
        res.append(k)
        n = (n-k)//256

    if n != 0: res.append(n)

    if len(res) > length_res:
        raise ValueError('{} cannot fit in a list of length {}'.format(n,length_res))
        return 1

    if len(res) < length_res:
        while len(res) < length_res:
            res.append(0)

    return res

def rad2deg(q):
    return q * 360 / (2*np.pi)

def deg2rad(q):
    return q / 360 * (2*np.pi)

def charlist2floatlist(l):
    res = []
    for x in l:
        try:
            res.append(float(x))
        except:
            res.append(0)

    return res

def floatlist2charlist(l):
    res = []
    for x in l:
        try:
            res.append('{}'.format(x))
        except:
            res.append('?')

    return res


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])