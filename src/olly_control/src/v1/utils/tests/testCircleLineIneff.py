from math import sqrt
from random import randint

from importlib.machinery import SourceFileLoader
circleLineIntersectionPath = SourceFileLoader('circleLineIntersectionPath','../circleLineIntersectionPath.py').load_module()
from circleLineIntersectionPath import circle_line_intersection


tol = 1e-5

# test varargin
assert (circle_line_intersection(1, 1, 2, -10, 1, 10, 1) == circle_line_intersection(1, 1, 2, -10, 1, 10, 1, 0, 0))
assert (circle_line_intersection(1, 1, 2, -10, 1, 10, 1) != circle_line_intersection(1, 1, 2, -10, 1, 10, 1, 3, 3))

# test alg on basic setup
xt, yt = circle_line_intersection(0, 0, 2, -10, 1, 10, 1, 10, 1)
assert (abs(xt - sqrt(3)) < tol)
assert (abs(yt - 1) < tol)

xt, yt = circle_line_intersection(0, 0, 2, -10, 1, 10, 1, -10, 1)
assert (abs(xt + sqrt(3)) < tol)
assert (abs(yt - 1) < tol)

xt, yt = circle_line_intersection(0, 0, 2, 1, 10, 1, -10, 1, 10)
assert (abs(xt - 1) < tol)
assert (abs(yt - sqrt(3)) < tol)

xt, yt = circle_line_intersection(0, 0, 2, 1, 10, 1, -10, 1, -10)
assert (abs(xt - 1) < tol)
assert (abs(yt + sqrt(3)) < tol)

# test(xm, ym) transformation on random data rng default
#
N = 10000  # randomized test count
min = -100  # random range max
max = 100  # random range min
for i in range(N):
    xm = randint(min, max)
    ym = randint(min, max)
    l = randint(0, max)
    x0 = randint(min, max)
    y0 = randint(min, max)
    x1 = randint(min, max)
    y1 = randint(min, max)
    xp = randint(min, max)
    yp = randint(min, max)

    xa, ya = circle_line_intersection(xm, ym, l, x0, y0, x1, y1, xp, yp)
    xb, yb = circle_line_intersection(0, 0, l, x0 - xm, y0 - ym, x1 - xm, y1 - ym, xp - xm, yp - ym)

    if xa is not None:
        assert (xa == xb + xm)
        assert (ya == yb + ym)

print('All Tests Passed.')
