import numpy


def trilateration(x1, y1, x2, y2, x3, y3, r1, r2, r3):

    x1, y1 = 1, 0
    x2, y2 = 1, 2
    x3, y3 = 3, 1
    r1, r2, r3 = 1, 1, 2

    P1 = numpy.array([x1, y1])
    P2 = numpy.array([x2, y2])
    P3 = numpy.array([x3, y3])

    ex = (P2 - P1)/(numpy.linalg.norm(P2 - P1))
    i = numpy.dot(ex, P3 - P1)
    ey = (P3 - P1 - i*ex)/(numpy.linalg.norm(P3 - P1 - i*ex))
    d = numpy.linalg.norm(P2 - P1)
    j = numpy.dot(ey, P3 - P1)


    x = (pow(r1, 2) - pow(r2, 2) + pow(d, 2))/(2*d)
    y = ((pow(r1, 2) - pow(r3, 2) + pow(i, 2) + pow(j, 2))/(2*j)) - ((i/j)*x)


    ans = P1 + x*ex + y*ey



    return ans

if __name__ == '__main__':

    x1, y1 = 1, 0
    x2, y2 = 1, 2
    x3, y3 = 3, 1
    r1, r2, r3 = 1, 1, 2

    point = trilateration(x1, y1, x2, y2, x3, y3, r1, r2, r3)
    print(point)