from math import *
from numpy import *
import matplotlib.pyplot as plt


def main():
    # test_cd()
    # test_ci()
    # test_jacobiana()
    test_tra()


def format_m(m):
    for i in range(len(m)):
        for j in range(len(m[i])):
            m[i][j] = round(m[i][j], 2)


def test_cd():
    print("\nPrueba de la cinematica directa")
    # casos de la a2
    m_a = cin_dir(0, 0, 0, 0)
    format_m(m_a)
    p_a = [m_a[0][3], m_a[1][3], m_a[2][3]]
    o_a = [[m_a[0][0], m_a[0][1], m_a[0][2]],
           [m_a[1][0], m_a[1][1], m_a[1][2]],
           [m_a[2][0], m_a[2][1], m_a[2][2]]]
    print("a) q(0, 0, 0, 0):")
    print("     Posición: ")
    print(array(p_a))
    print("     Orientación: ")
    print(array(o_a))
    m_b = cin_dir(0, pi/2, 0, 0)
    format_m(m_b)
    p_b = [m_b[0][3], m_b[1][3], m_b[2][3]]
    o_b = [[m_b[0][0], m_b[0][1], m_b[0][2]],
           [m_b[1][0], m_b[1][1], m_b[1][2]],
           [m_b[2][0], m_b[2][1], m_b[2][2]]]
    print("b) q(0, pi/2, 0, 0):")
    print("     Posición: ")
    print(array(p_b))
    print("     Orientación: ")
    print(array(o_b))
    m_c = cin_dir(-pi/2, pi/2, 0, 50)
    format_m(m_c)
    p_c = [m_c[0][3], m_c[1][3], m_c[2][3]]
    o_c = [[m_c[0][0], m_c[0][1], m_c[0][2]],
           [m_c[1][0], m_c[1][1], m_c[1][2]],
           [m_c[2][0], m_c[2][1], m_c[2][2]]]
    print("c) q(-pi/2, pi/2, 0, 50):")
    print("     Posición: ")
    print(array(p_c))
    print("     Orientación: ")
    print(array(o_c))
    m_d = cin_dir(pi, 0, pi/2, 25)
    format_m(m_d)
    p_d = [m_d[0][3], m_d[1][3], m_d[2][3]]
    o_d = [[m_d[0][0], m_d[0][1], m_d[0][2]],
           [m_d[1][0], m_d[1][1], m_d[1][2]],
           [m_d[2][0], m_d[2][1], m_d[2][2]]]
    print("d) q(pi, 0, pi/2, 25):")
    print("     Posición: ")
    print(array(p_d))
    print("     Orientación: ")
    print(array(o_d))

    # mas casos
    m_p1 = cin_dir(-pi / 4, pi / 4, 0, 10)
    format_m(m_p1)
    p_p1 = [m_p1[0][3], m_p1[1][3], m_p1[2][3]]
    o_p1 = [[m_p1[0][0], m_p1[0][1], m_p1[0][2]],
            [m_p1[1][0], m_p1[1][1], m_p1[1][2]],
            [m_p1[2][0], m_p1[2][1], m_p1[2][2]]]
    print("Prueba1) q(-pi/4, pi/4, 0, 10):")
    print("     Posición: ")
    print(array(p_p1))
    print("     Orientación: ")
    print(array(o_p1))
    m_p2 = cin_dir(0, pi / 4, pi / 6, 0)
    format_m(m_p2)
    p_p2 = [m_p2[0][3], m_p2[1][3], m_p2[2][3]]
    o_p2 = [[m_p2[0][0], m_p2[0][1], m_p2[0][2]],
            [m_p2[1][0], m_p2[1][1], m_p2[1][2]],
            [m_p2[2][0], m_p2[2][1], m_p2[2][2]]]
    print("Prueba2) q(0, pi/4, pi/6, 0):")
    print("     Posición: ")
    print(array(p_p2))
    print("     Orientación: ")
    print(array(o_p2))


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
    # características del robot
    brazo = 105
    antebrazo = 75
    # valor de la articulacion prismatica
    d3 = [50 - z, 50 - z]
    # distancia sin tener en cuenta z de la mano y el origen
    r = sqrt(x**2 + y**2)
    # angulo que forma el vector posicion
    epsilon = atan2(y, x)
    # angulo entre el vector posicion y el brazo
    aux1 = brazo**2 + r**2 - antebrazo**2
    aux2 = 2 * r * brazo
    fi = acos(aux1 / aux2)
    # valores posibles de la articulacion de giro 1
    th1 = [epsilon - fi, epsilon + fi]
    # valores posibles de la articulacion de giro 2
    th_aux = asin((r * sin(fi)) / antebrazo)
    if r < sqrt(brazo ** 2 + antebrazo ** 2):
        th_aux = pi - th_aux
    th2 = [th_aux, -th_aux]
    # valores posibles de la articulacion de giro 4
    th4 = [orientacion - th1[0] - th2[0], orientacion - th1[1] - th2[1]]
    return [th1, th2, th4, d3]


def test_ci():
    print("\nPrueba de la cinematica inversa")
    # casos de la a2
    conf_a = cin_inv(180, 0, 50, 0)
    format_m(conf_a)
    conf_b = cin_inv(105, 75, 50, pi/2)
    format_m(conf_b)
    conf_c = cin_inv(75, -105, 0, 0)
    format_m(conf_c)
    conf_d = cin_inv(-180, 0, 25, (3*pi)/2)
    format_m(conf_d)

    print("a) posicion (180, 0, 50), orientacion (0), q(0, 0, 0, 0):")
    print(array(conf_a))
    print("b) posicion (105, 75, 50), orientacion (pi/2), q(0, pi/2, 0, 0):")
    print(array(conf_b))
    print("c) posicion (75, -105, 0), orientacion (0), q(-pi/2, pi/2, 0, 50):")
    print(array(conf_c))
    print("d) posicion (-180, 0, 25), orientacion (-pi/2), q(pi, 0, pi/2, 25):")
    print(array(conf_d))
    # mas casos
    conf_p1 = cin_inv(149.25, -74.25, 40, 0)
    format_m(conf_p1)
    conf_p2 = cin_inv(158.03, 53.03, 50, (5*pi)/12)
    format_m(conf_p2)

    print("Prueba1) posicion (149.25, -74.25, 40), orientacion (0), q(-pi/4, pi/4, 0, 10):")
    print(array(conf_p1))
    print("Prueba2) posicion (158.03, 53.03, 50), orientacion (5pi/12), q(0, pi/4, pi/6, 0):")
    print(array(conf_p2))


def jacobiana(th1, th2, th4, d3):
    j = [[-75*sin(th1 + th2) - 105*sin(th1), -75*sin(th1 + th2), 0, 0],
         [75*cos(th1 + th2) + 105*cos(th1), 75*cos(th1 + th2), 0, 0],
         [0, 0, -1, 0],
         [1, 1, 0, 1]]
    return j


def test_jacobiana():
    print("\nPruebas de la jacobiana")
    # casos de la a2
    mja = jacobiana(0, -pi/2, pi/2, 25)
    format_m(mja)
    mjb = jacobiana(pi/4, -pi/4, 0, 0)
    format_m(mjb)

    print("a) q(0, -pi/2, pi/2, 25):")
    print(array(mja))
    print("b) q(pi/4, -pi/4, 0, 0):")
    print(array(mjb))

    # mas casos
    mjp1 = jacobiana(-pi/2, pi/2, -pi/2, 0)
    format_m(mjp1)
    mjp2 = jacobiana(pi/4, -pi/2, 0, 0)
    format_m(mjp2)
    mjp3 = jacobiana(0, pi/2, pi/4, 30)
    format_m(mjp3)

    print("Prueba1) q(-pi/2, pi/2, -pi/2, 0):")
    print(array(mjp1))
    print("Prueba2) q(pi/4, -pi/2, 0, 0):")
    print(array(mjp2))
    print("Prueba3) q(0, pi/2, pi/4, 30):")
    print(array(mjp3))


def trayectoria(x0, v_x, a_x, y0, v_y, a_y, z0, v_z, a_z, r, ang0, v_ang):
    # realiza trayectorias definidas por las ecuaciones paramétricas:
    # x = x0 + v_x * t + a_x * t^2 + r * sen(ang0 + v_ang * t)
    # y = y0 + v_y * t + a_y * t^2 + r * cos(ang0 + v_ang * t)
    # z = z0 + v_z * t + a_z * t^2
    # sirve para rectas, curvas parabolicas, circulos y mezclas
    puntos = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
    configuraciones = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
    # inicializa las posiciones de los puntos a seguir
    for t in range(0, 21):
        puntos[0][t] = x0 + v_x * t + a_x * t**2 + r * sin(ang0 + v_ang * t)
        puntos[1][t] = y0 + v_y * t + a_y * t**2 + r * cos(ang0 + v_ang * t)
        puntos[2][t] = z0 + v_z * t + a_z * t**2
    for t in range(0, 21):
        # calculo la orientacion del eje x: angulo formado por dx/dt y dy/dt
        dx = v_x + a_x * t
        dy = v_y + a_y * t
        angulo = atan2(dy, dx)
        # calculo las coordenadas posibles de las articulaciones
        config = cin_inv(puntos[0][t], puntos[1][t], puntos[2][t], angulo)
        # escoge una de las coordenadas posibles de las articulaciones
        aux = 1
        # si es el primer punto se escoge segun la posicion de la mano
        if t == 0:
            if puntos[1][t] > 0:
                aux = 0
            for i in range(len(config)):
                configuraciones[i][t] = config[i][aux]
        # para el resto de puntos escoge la configuracion
        #   con theta1 mas parecido a la configuracion anterior
        else:
            if abs(config[0][0] - configuraciones[0][t-1]) < abs(config[0][1] - configuraciones[0][t-1]):
                aux = 0
            for i in range(len(config)):
                configuraciones[i][t] = config[i][aux]
    return configuraciones


def test_tra():
    print("\nPruebas de trayectorias")
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # trayectoria roja: recta entre (-75, 105, 50) y (85, 125, 10)
    config = trayectoria(-75, 8, 0, 105, 1, 0, 50, -2, 0, 0, 0, 0)
    print("Serie de configuraciones de la recta")
    print(array(config))
    for i in range(0, 21):
        matriz = cin_dir(config[0][i], config[1][i], config[2][i], config[3][i])
        pos = [matriz[0][3], matriz[1][3], matriz[2][3]]
        ax.scatter(pos[0], pos[1], pos[2], c='r', marker='o')
    # trayectoria azul: parábola entre (-75, 105, 0) y (85, 105, 0)
    config = trayectoria(-75, 8, 0, 105, 10, -0.5, 0, 10, -0.5, 0, 0, 0)
    print("Serie de configuraciones de la parábola")
    print(array(config))
    for i in range(0, 21):
        matriz = cin_dir(config[0][i], config[1][i], config[2][i], config[3][i])
        pos = [matriz[0][3], matriz[1][3], matriz[2][3]]
        ax.scatter(pos[0], pos[1], pos[2], c='b', marker='^')
    # trayectoria verde: circulo de radio 30 alrededor del punto (10, 140, 10)
    config = trayectoria(10, 0, 0, 140, 0, 0, 10, 0, 0, 30, pi, pi/10)
    print("Serie de configuraciones del circulo")
    print(array(config))
    for i in range(0, 21):
        matriz = cin_dir(config[0][i], config[1][i], config[2][i], config[3][i])
        pos = [matriz[0][3], matriz[1][3], matriz[2][3]]
        ax.scatter(pos[0], pos[1], pos[2], c='g', marker='s')
    # dibuja las trayectorias con matplotlib
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()
