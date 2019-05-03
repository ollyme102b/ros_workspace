import numpy as np


def circle_line_intersection(xm, ym, l, x0, y0, x1, y1, xp=0, yp=0):

    """
    computes the optimal folly position

    :param xm: molly x position
    :param ym: molly y position
    :param l: length of constraint line between molly and folly
    :param x0: x position of zero-end of folly path constraint
    :param y0: y position of zero-end of folly path constraint
    :param x1: x position of one-end of folly path constraint
    :param y1: y position of one-end of folly path constraint
    :param xp: folly x position from previous step
    :param yp: folly y position from previous step
    :return: xf: folly x optimal position
    :return: yf: folly y optimal position
    """

    # center problem around xm and ym
    x0 = x0 - xm
    y0 = y0 - ym
    x1 = x1 - xm
    y1 = y1 - ym
    xp = xp - xm
    yp = yp - ym

    if x0 - x1 == 0:  # slope undefined case
        xfs = np.array([x0, x0])  # solution must have same x value
        temp = np.sqrt(l ** 2 - x0 ** 2)

        if np.isnan(temp):
            return None, None

        yfs = np.array([temp, -temp])  # solve circle intersect vertical line y value
    else:  # slope defined case
        m = (y1 - y0) / (x1 - x0)  # slope
        b = y0 - m * x0  # y intercept

        xfs = np.roots([1 + m ** 2, 2 * m * b, b ** 2 - l ** 2])  # solve circle intersect line x values
        yfs = m * xfs + b  # deduce y values from x values

    # if solution not feasible
    if np.iscomplex(xfs[0]):
        return None, None

    # return solution closest to previous position
    if (xfs[0] - xp) ** 2 + (yfs[0] - yp) ** 2 < (xfs[1] - xp) ** 2 + (yfs[1] - yp) ** 2:
        i = 0
    else:
        i = 1

    xf = xfs[i] + xm
    yf = yfs[i] + ym

    return xf, yf


def circle_line_intersection_path(m_pos, l, l0, l1, d_m_pos, n, f_pos=np.array([0, 0])):
    """
    :param m_pos: molly position vector
    :param l: length of constraint line between molly and folly
    :param l0: position vector of start of path line constraint
    :param l1: position vector of end of path line constraint
    :param d_m_pos: deviation of molly per time step
    :param n: path horizon
    :param f_pos: folly position vector
    :return: folly path
    """

    folly_path = np.zeros((2, n))
    xp = f_pos[0]
    yp = f_pos[1]

    for i in range(n):
        xf, yf = circle_line_intersection(m_pos[0], m_pos[1], l, l0[0], l0[1], l1[0], l1[1], xp=xp, yp=yp)

        # add time step path to output path arra
        # update folly position for next iteration
        if xf is not None:
            folly_path[0, i] = xf
            xp = xf
        else:
            folly_path[0, i] = xp

        if yf is not None:
            yp = yf
            folly_path[1, i] = yf
        else:
            folly_path[1, i] = yp


        # update molly position for next iteration
        m_pos = m_pos + d_m_pos

    return folly_path
