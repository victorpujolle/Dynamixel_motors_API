import numpy as np

def pseudo_conversion(n):
    """
    :param n: integer
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

    return res