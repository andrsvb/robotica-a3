# This is a sample Python script.

from numpy import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def main():
    test_cd()
    # test_ci()


def test_cd():
    # casos de la a2
    m_a = cin_dir(0, 0, 0, 0)
    p_a = [round(m_a[0][3], 2), round(m_a[1][3], 2), round(m_a[2][3], 2)]
    o_a = [[round(m_a[0][0], 2), round(m_a[0][1], 2), round(m_a[0][2], 2)],
           [round(m_a[1][0], 2), round(m_a[1][1], 2), round(m_a[1][2], 2)],
           [round(m_a[2][0], 2), round(m_a[2][1], 2), round(m_a[2][2], 2)]]
    print("a) q(0, 0, 0, 0):")
    print("     Posición: ")
    print(array(p_a))
    print("     Orientación: ")
    print(array(o_a))
    m_b = cin_dir(0, pi/2, 0, 0)
    p_b = [round(m_b[0][3], 2), round(m_b[1][3], 2), round(m_b[2][3], 2)]
    o_b = [[round(m_b[0][0], 2), round(m_b[0][1], 2), round(m_b[0][2], 2)],
           [round(m_b[1][0], 2), round(m_b[1][1], 2), round(m_b[1][2], 2)],
           [round(m_b[2][0], 2), round(m_b[2][1], 2), round(m_b[2][2], 2)]]
    print("b) q(0, pi/2, 0, 0):")
    print("     Posición: ")
    print(array(p_b))
    print("     Orientación: ")
    print(array(o_b))
    m_c = cin_dir(-pi/2, pi/2, 0, 50)
    p_c = [round(m_c[0][3], 2), round(m_c[1][3], 2), round(m_c[2][3], 2)]
    o_c = [[round(m_c[0][0], 2), round(m_c[0][1], 2), round(m_c[0][2], 2)],
           [round(m_c[1][0], 2), round(m_c[1][1], 2), round(m_c[1][2], 2)],
           [round(m_c[2][0], 2), round(m_c[2][1], 2), round(m_c[2][2], 2)]]
    print("c) q(-pi/2, pi/2, 0, 50):")
    print("     Posición: ")
    print(array(p_c))
    print("     Orientación: ")
    print(array(o_c))
    m_d = cin_dir(pi, 0, pi/2, 25)
    p_d = [round(m_d[0][3], 2), round(m_d[1][3], 2), round(m_d[2][3], 2)]
    o_d = [[round(m_d[0][0], 2), round(m_d[0][1], 2), round(m_d[0][2], 2)],
           [round(m_d[1][0], 2), round(m_d[1][1], 2), round(m_d[1][2], 2)],
           [round(m_d[2][0], 2), round(m_d[2][1], 2), round(m_d[2][2], 2)]]
    print("a) q(pi, 0, pi/2, 25):")
    print("     Posición: ")
    print(array(p_d))
    print("     Orientación: ")
    print(array(o_d))


def cin_dir(th1, th2, th4, d3):
    #   Tabla DyH:
    #   i   alfa    a       theta   d
    #   1   0       105     th1     120
    #   2   0       75      th2     0
    #   3   0       0       0       -70-d3
    #   4   0       0       th4     0
    m01 = mat_dh(0, 105, th1, 120)
    m12 = mat_dh(0, 75, th2, 0)
    m23 = mat_dh(0, 0, 0, -70-d3)
    m34 = mat_dh(0, 0, th4, 0)
    m04 = dot(dot(dot(m01, m12), m23), m34)     # M01*M12*M23*M34
    return m04


def mat_dh(alfa, a, theta, d):
    mdh = [[cos(theta), -sin(theta) * cos(alfa), sin(theta) * sin(alfa), a * cos(theta)],
           [sin(theta), cos(theta) * cos(alfa), -cos(theta) * sin(alfa), a * sin(theta)],
           [0, sin(alfa), cos(alfa), d],
           [0, 0, 0, 1]]
    return mdh


def cin_inv(x, y, z, orientacion):
    # orientacion se refiere al angulo que forman los ejes x0 y x4
    brazo = 105
    antebrazo = 75
    d3 = 50 - z
    r = sqrt(x**2 + y**2)
    # arctan devuelve el angulo entre entre -pi/2 y pi/2
    # por lo tanto hay que corregir cuando el angulo esta entre pi/2 y -pi/2
    epsilon = 0
    if x >= 0:
        epsilon = arctan(x/y)
    else:
        epsilon = arctan(x/y) + pi  # esta bien? radianes
    aux1 = brazo**2 + r**2 - antebrazo**2
    aux2 = 2 * r * brazo
    fi = arccos(aux1 / aux2)      # fi siempre esta entre 0 y pi/2
    th1 = [epsilon - fi, epsilon + fi]
    th_aux = arcsin((r * sin(fi)) / antebrazo)
    th2 = [th_aux, -th_aux]
    th4 = orientacion - th1 - th2
    return [th1, th2, th4, d3]


def test_ci():
    # casos de la a2
    conf_a = cin_inv(0, 0, 0, 0)
    conf_b = cin_inv(0, math.pi/2, 0, 0)
    conf_c = cin_inv(-math.pi/2, math.pi/2, 0, 50)
    conf_d = cin_inv(math.pi, 0, math.pi/2, 25)
    # mas casos


def jacobiana(th1, th2, th4, d3):
    j = [[-75*sin(th1 + th2) - 105*sin(th1), -75*sin(th1 + th2), 0, 0],
         [75*cos(th1 + th2) - 105*cos(th1), 75*cos(th1 + th2), 0, 0],
         [0, 0, -1, 0],
         [1, 1, 0, 1]]
    return j


def trayectoria():
    a = 0
    a = 0


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
