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