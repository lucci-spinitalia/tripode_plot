# -*- coding: utf-8 -*-
# --cython: profile=True
# --cython: linetrace=True

# compile.sh
# filename=$(basename "$1")
# cython -a $filename
# gcc -shared -pthread -fPIC -fwrapv -O3 -ffast-math -Wall -fno-strict-aliasing -I/usr/include/python2.7 -o ${filename%.*}.so ${filename%.*}.c
# cp kinematic_cy.html /home/henry/tripode/

from libc.math cimport sin, cos, sqrt, asin, M_PI

cdef struct plane3d_t:
    double x[3]
    double y[3]
    double z[3]

cdef:
    plane3d_t base
    plane3d_t roof
    double real_height, base_length
    double alphas_start[3]
    double alphas_temp[3]
    double alphas_last[3]
    double motor_positions[3]
    double nodes_height[3]
    double alphas_sin[3]
    double alphas_cos[3]
    double s2_sin30
    double s2_cos30
    double s3_sin30
    double s3_cos30
    double alpha_limit_r
    double step_alpha_base_r
    long cycles
    double ke, err_limit
    double roll, pitch, yaw
    double roll_r, pitch_r, yaw_r

def init():
    """
    Ottimizzazioni apportate:
        - tipizzazione dei dati: 10x
        - math lib: 50x
        - rimozione calcoli non utili
        - eliminazione passaggio dei parametri
        - precalcolo coseni

    """

    cdef:
        double base_radius
        double alpha_limit
        double step_alpha_base
        double base_height

    global real_height
    global ke
    global err_limit
    global alpha_limit_r
    global base_length
    global base
    global step_alpha_base_r
    global alphas_temp
    global motor_positions
    global nodes_height
    global alphas_start
    global alphas_last

    # COSTANTI ---------------------------------------------------------------------------------------------------

    base_radius = 0.705     # Raggio dei centri dei tre pistoni
    real_height = 1.685     # Distanza tre i centri cerniera-sfera sullo 0 ( 400mm )

    alpha_limit = 10.0      # Massima escursione dei pistoni in +/- 10 gradi
    step_alpha_base = 0.1   # Gli incrementi angolari partono da 0.1 gradi

    ke = 150                # Guadagno nella ricerca dell'angolo
    err_limit = 0.00015     # Errore sotto il quale la soluzione si considera esatta ( 0,1mm )

    # COSTANTI CALCOLATE -----------------------------------------------------------------------------------------

    # Calcolo le costanti in radianti
    alpha_limit_r = -alpha_limit / 180.0 * M_PI

    # Calcolo delle coordinate dei vertici della base
    base_length = base_radius * sqrt(3)
    base_height = 3 * base_radius / 2
    base.x[1] = base_height
    base.y[1] = base_length / 2
    base.z[1] = base_length / 2
    base.x[2] = base_height
    base.y[2] = -base_length / 2
    base.z[2] = -base_length / 2

    # Minimo incremento dell'angolo
    step_alpha_base_r = step_alpha_base / 180.0 * M_PI

    # Valori temporanei contenenti gli angoli di base iterati
    alphas_temp[:] = [0.0, 0.0, 0.0]

    # Posizione in step dei motori
    motor_positions[:] = [0.0, 0.0, 0.0]

    # Posizione dei tre pistoni in metri da terra
    nodes_height[:] = [0.0, 0.0, 0.0]

    # Angolo di partenza delle ricerche
    alphas_start[:] = [alpha_limit_r, alpha_limit_r, alpha_limit_r]
    alphas_last[:] = [alpha_limit_r, alpha_limit_r, alpha_limit_r]

cdef inline double distance_12():
    """Calcolo della distanta tra i centri sfera dei pistoni 1 e 2

    In ingresso vengono forniti i tre angoli ipotetici, e le tre altezze dei pistoni

    """

    global roof
    global nodes_height
    global alphas_cos
    global alphas_sin
    global s2_sin30
    global s2_cos30

    roof.z[0] = nodes_height[0] * alphas_cos[0]
    roof.x[0] = nodes_height[0] * alphas_sin[0]

    roof.z[1] = nodes_height[1] * alphas_cos[1]
    roof.x[1] = base.x[1] - s2_sin30
    roof.y[1] = base.y[1] - s2_cos30

    return sqrt(
        ((roof.x[1] - roof.x[0]) ** 2) +
        (roof.y[1] ** 2) +
        ((roof.z[1] - roof.z[0]) ** 2))

cdef inline double distance_23():
    """Calcolo della distanta tra i centri sfera dei pistoni 2 e 3

    In ingresso vengono forniti i tre angoli ipotetici, e le tre altezze dei pistoni

    """

    global roof
    global nodes_height
    global alphas_cos
    global alphas_sin
    global s2_sin30
    global s2_cos30
    global s3_sin30
    global s3_cos30

    #roof.z[1] = nodes_height[1] * alphas_cos[1]
    #roof.x[1] = base.x[1] - s2_sin30
    #roof.y[1] = base.y[1] - s2_cos30

    roof.z[2] = nodes_height[2] * alphas_cos[2]
    roof.x[2] = base.x[2] - s3_sin30
    roof.y[2] = base.y[2] + s3_cos30

    return sqrt(
        ((roof.x[2] - roof.x[1]) ** 2) +
        ((roof.y[1] - roof.y[2]) ** 2) +
        ((roof.z[2] - roof.z[1]) ** 2))

cdef inline double distance_13():
    """Calcolo della distanta tra i centri sfera dei pistoni 1 e 3

    In ingresso vengono forniti i tre angoli ipotetici, e le tre altezze dei pistoni

    """

    global roof
    global nodes_height
    global alphas_cos
    global alphas_sin
    global s3_sin30
    global s3_cos30

    #roof.z[0] = nodes_height[0] * alphas_cos[0]
    #roof.x[0] = nodes_height[0] * alphas_sin[0]

    #roof.z[2] = nodes_height[2] * alphas_cos[2]
    #roof.x[2] = base.x[2] - s3_sin30
    #roof.y[2] = base.y[2] + s3_cos30

    return sqrt(
        ((roof.x[2] - roof.x[0]) ** 2) +
        (roof.y[2] ** 2) +
        ((roof.z[2] - roof.z[0]) ** 2))

cdef inline void update_alfas_precalc():

    global alphas_cos
    global alphas_sin
    global s2_sin30
    global s2_cos30
    global s3_sin30
    global s3_cos30

    cdef double s2, s3

    alphas_sin[0] = sin(alphas_temp[0])
    alphas_cos[0] = cos(alphas_temp[0])
    alphas_sin[1] = sin(alphas_temp[1])
    alphas_cos[1] = cos(alphas_temp[1])
    alphas_sin[2] = sin(alphas_temp[2])
    alphas_cos[2] = cos(alphas_temp[2])
    s2 = nodes_height[1] * alphas_sin[1]
    s2_sin30 = s2 / 2
    s2_cos30 = s2 / 1.154700538379252
    s3 = nodes_height[2] * alphas_sin[2]
    s3_sin30 = s3 / 2
    s3_cos30 = s3 / 1.154700538379252

cdef int search_base_angles(motor_positions):

    global alphas_temp
    global alphas_last
    global base_length
    global nodes_height
    global real_height
    global step_alpha_base_r
    global cycles

    # Errore nelle soluzioni
    cdef:
        double err[3]
        int cycle_limit
        int n, j, i
        double step_alpha[3]

    cycle_limit = 1000

    # Angolo di inizio ricerca
    alphas_temp[0] = alphas_last[0]
    alphas_temp[1] = alphas_last[1]
    alphas_temp[2] = alphas_last[2]
    update_alfas_precalc()

    # Altezze reali degli attuatori
    nodes_height[0] = motor_positions[0] + real_height
    nodes_height[1] = motor_positions[1] + real_height
    nodes_height[2] = motor_positions[2] + real_height

    # Incrementi degli angoli di base da 1/10 di grado
    step_alpha[:] = [step_alpha_base_r, step_alpha_base_r, step_alpha_base_r]

    # Numero di cicli eseguiti
    cycles = 0

    # Calcolo la condizione iniziale
    err[0] = distance_12() - base_length
    step_alpha[1] = err[0] * ke * step_alpha_base_r
    err[1] = distance_23() - base_length
    step_alpha[2] = err[1] * ke * step_alpha_base_r
    err[2] = distance_13() - base_length
    step_alpha[0] = err[2] * ke * step_alpha_base_r

    # Ciclo per la variazione di alfa1
    for i in range(cycle_limit):

        # Incremento alfa1 ed azzero alfa2
        alphas_temp[0] += step_alpha[0]
        alphas_temp[1] = alphas_start[1]
        update_alfas_precalc()

        for j in range(cycle_limit):

            #next_iteration(alpha, step_alpha, i, j, n, err)
            cycles += 1

            if cycles > cycle_limit:
                # error_description = "Maximum number of cycles executed, no solution found!"
                return -2

            # Incremento alfa1 ed azzero alfa2
            alphas_temp[1] += step_alpha[1]
            update_alfas_precalc()

            # Se supero l'angolo limite
            # Partendo da -10 ( -0.17 ), non devo superare 10 ( 0.17 )
            if alphas_temp[1] > -alpha_limit_r:

                # Angolo non trovato
                step_alpha[1] = step_alpha_base_r
                step_alpha[0] = err[0] * ke * step_alpha_base_r
                alphas_start[1] = -alpha_limit_r - 2 * step_alpha[1]
                break

            err[0] = distance_12() - base_length
            step_alpha[1] = err[0] * ke * step_alpha_base_r

            if abs(err[0]) < err_limit:

                # Trovato il minimo
                alphas_start[1] = alphas_temp[1]
                step_alpha[1] = step_alpha_base_r

                for n in range(cycle_limit):

                    #next_iteration(alpha, step_alpha, i, j, n, err)
                    cycles += 1

                    if cycles > cycle_limit:
                        # error_description = "Maximum number of cycles executed, no solution found!"
                        return -2

                    alphas_temp[2] += step_alpha[2]
                    update_alfas_precalc()
                    err[1] = distance_23() - base_length
                    step_alpha[2] = err[1] * ke * step_alpha_base_r

                    if abs(err[1]) < err_limit:

                        step_alpha[2] = step_alpha_base_r
                        err[2] = distance_13() - base_length
                        step_alpha[0] = err[2] * ke * step_alpha_base_r

                        if abs(err[2]) < err_limit:

                            # Trovatas la soluzione
                            alphas_last[0] = alphas_temp[0]
                            alphas_last[1] = alphas_temp[1]
                            alphas_last[2] = alphas_temp[2]
                            return 0

                        # Next n!!!
                        break

                # Next i!!!
                break


cpdef find_plane_angles(motor_positions):

    global roof
    global roll
    global pitch
    global yaw
    global roll_r
    global pitch_r
    global yaw_r

    cdef:
        double center_point_x
        double center_point_y
        double center_point_z
        int i, j
        double base_r[3][3]
        double mat_rot[3][3]

    # Calcolo il punto mediano tra i vertici 2 e 3
    center_point_x = (roof.x[1] + roof.x[2]) / 2
    center_point_y = (roof.y[1] + roof.y[2]) / 2
    center_point_z = (roof.z[1] + roof.z[2]) / 2

    # Matrice con i tre punti del piano finora calcolati da distante_12/23/13 ed un
    # quarto punto mediano rispetto a 2 e 3

    # Questa e' l'inizializzazione della matrice con le posizioni
    base_r[0][:] = [roof.x[0] - center_point_x,
                    roof.y[0] - center_point_y,
                    roof.z[0] - center_point_z]
    base_r[1][:] = [roof.x[1] - center_point_x,
                    roof.y[1] - center_point_y,
                    roof.z[1] - center_point_z]
    base_r[2][:] = [0.0, 0.0, 0.0]

    # Questa e' la costruzione di una matrice di rotazione
    mat_rot[0][:] = [0.0, 0.0, 0.0]
    mat_rot[1][:] = [0.0, 0.0, 0.0]
    mat_rot[2][:] = [0.0, 0.0, 0.0]

    # Questo è il calcolo della matrice di rotazione
    for i in range(0, 2):
        mr = sqrt((base_r[i][0] ** 2) + (base_r[i][1] ** 2) + (base_r[i][2] ** 2))
        for j in range(0, 3):
            base_r[i][j] = base_r[i][j] / mr
            mat_rot[j][i] = base_r[i][j]
    base_r[2][0] = +base_r[1][1] * base_r[0][2] - base_r[0][1] * base_r[1][2]
    base_r[2][1] = -base_r[1][0] * base_r[0][2] + base_r[0][0] * base_r[1][2]
    base_r[2][2] = +base_r[1][0] * base_r[0][1] - base_r[0][0] * base_r[1][1]
    for i in range(0, 3):
        mat_rot[i][2] = base_r[2][i]

    # Dalla matrice di rotazione, ricavo per ispezione la terna di angoli di Tait-Bryan
    k17 = mat_rot[2][0]
    k16 = mat_rot[1][0]
    l17 = mat_rot[2][1]
    m20 = asin(k17)
    i23 = cos(m20)
    i24 = k16 / i23
    i25 = l17 / i23
    m19 = asin(i24)
    yaw_r = m19 + motor_positions[3]
    pitch_r = asin(k17)
    roll_r = asin(i25)
    roll = roll_r / M_PI * 180.0
    pitch = pitch_r / M_PI * 180.0
    yaw = yaw_r / M_PI * 180.0

cpdef angles_to_avionics(zyx3, zyx2, zyx1):
    """Converte una terna rotazionale da interna in avionica

    Voglio convertire una termina angolare:

      Z1Y2X3 -> Y1X2Z3

    Possiedo i tre angoli di Z1Y2X3, che sono diversi dai tre di Y1X2Z3.

    Procedo per ispezione tra le matrici:

            INPUT
                     |      c1c2         c1s2s3 - s1c3    s1s3 + c1s2c3  |
            Z1Y2X3 = |      s1c2         c1c3 + s1s2s3    s1s2c3 - c1s3  |
-                     |      -s2               c2s3            c2c3       |

            OUTPUT
                     |  c1c3 + s1s2s3    s1s2c3 - c1s3        s1c2       |
            Y1X2Z3 = |      c2s3              c2c3            -s2        |
                     |  c1s2s3 - s1c3    c1s2c3 + s1s3        c1c2       |

    """

    cdef:
        double s1c2_yxz
        double s2_yxz
        double c2s3_yxz
        double c2_yxz
        double rx_yxz_r
        double ry_yxz_r
        double rz_yxz_r
        double rx_yxz
        double ry_yxz
        double rz_yxz

    # Ora calcolo i termini in funzione degli angoli forniti
    s1c2_yxz = (sin(zyx1) * sin(zyx3)) + (cos(zyx1)* sin(zyx2) * cos(zyx3))
    s2_yxz = (cos(zyx1) * sin(zyx3)) - (sin(zyx1) * sin(zyx2) * cos(zyx3))
    c2s3_yxz = sin(zyx1) * cos(zyx2)

    # Ora trovo gli angoli
    rx_yxz_r = asin(s2_yxz)
    c2_yxz = cos(rx_yxz_r)
    ry_yxz_r = asin(s1c2_yxz / c2_yxz)
    rz_yxz_r = asin(c2s3_yxz / c2_yxz)
    rx_yxz = rx_yxz_r / M_PI * 180.0
    ry_yxz = ry_yxz_r / M_PI * 180.0
    rz_yxz = rz_yxz_r / M_PI * 180.0

    return [rx_yxz, ry_yxz, rz_yxz, rx_yxz_r, ry_yxz_r, rz_yxz_r]

cpdef angles_to_internals(rx_yxz, ry_yxz, rz_yxz):
    """Converte una terna rotazionale xyz da avionica ad interna, zyx

    TODO: NON Funziona, o meglio restituisce un risultato diverso dal python

    Gli angoli in ingresso sono in gradi.

    Voglio convertire una termina angolare:

      Y1X2Z3-> Z1Y2X3

    Possiedo i tre angoli di X1Y2Z3, che sono diversi dai tre di Z1Y2X3.

    Procedo per ispezione tra le matrici:

            INPUT
                     |  c1c3 + s1s2s3    s1s2c3 - c1s3        s1c2       |
            Y1X2Z3 = |      c2s3              c2c3            -s2        |
                     |  c1s2s3 - s1c3    c1s2c3 + s1s3        c1c2       |

            OUTPUT
                     |      c1c2         c1s2s3 - s1c3    s1s3 + c1s2c3  |
            Z1Y2X3 = |      s1c2         c1c3 + s1s2s3    s1s2c3 - c1s3  |
                     |      -s2               c2s3            c2c3       |

    """

    cdef:
        double rx_yxz_r
        double ry_yxz_r
        double rz_yxz_r
        double c2s3_zyx
        double s2_zyx
        double s1c2_zyx
        double c2_zyx
        double zyx3_r
        double zyx2_r
        double zyx1_r
        double zyx3
        double zyx2
        double zyx1

    # Converto gli angoli in ingresso in gradi
    rx_yxz_r = (rx_yxz / 180.0) * M_PI
    ry_yxz_r = (ry_yxz / 180.0) * M_PI
    rz_yxz_r = (rz_yxz / 180.0) * M_PI

    # Ora calcolo i termini in funzione degli angoli forniti
    c2s3_zyx = (cos(ry_yxz_r) * sin(rx_yxz_r) * cos(rz_yxz_r)) + (sin(ry_yxz_r) * sin(rz_yxz_r))
    s2_zyx = (sin(ry_yxz_r) * cos(rz_yxz_r)) - (cos(ry_yxz_r) * sin(rx_yxz_r) * sin(rz_yxz_r))
    s1c2_zyx = cos(rx_yxz_r) * sin(rz_yxz_r)

    # Ora trovo gli angoli
    zyx2_r = asin(s2_zyx)
    c2_zyx = cos(zyx2_r)
    zyx1_r = asin(s1c2_zyx / c2_zyx)
    zyx3_r = asin(c2s3_zyx / c2_zyx)
    zyx3 = (zyx3_r / M_PI) * 180.0
    zyx2 = (zyx2_r / M_PI) * 180.0
    zyx1 = (zyx1_r / M_PI) * 180.0

    return [zyx3, zyx2, zyx1, zyx3_r, zyx2_r, zyx1_r]

def search_angles(motor_positions):

    global alphas_last
    global roll
    global pitch
    global yaw
    global roll_r
    global pitch_r
    global yaw_r
    global cycles

    cdef int res

    res = search_base_angles(motor_positions)

    if res == 0:
        find_plane_angles(motor_positions)
        return [res, roll, pitch, yaw, roll_r, pitch_r, yaw_r, cycles]
    else:
        return [res, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cycles]

def search_angles_test(heights, rotation):

    cdef int i

    for i in range(9999):
        search_angles(heights, rotation)

    return search_angles(heights, rotation)


def search():

    cdef:
        int i
        double res_12
        double res_23
        double res_13

    nodes_height[:] = [0.01, 0.01, 0.01]

    for i in range(1000000):
        distance_12()
        distance_23()
        distance_13()
        nodes_height[1] += 0.000000001
        nodes_height[1] -= 0.000000001
        update_alfas_precalc()

def test_distance():

    cdef:
        double res_12
        double res_23
        double res_13

    # Provo le funzioni di calcolo delle distanze
    alphas_temp[:] = [0.3, 0.4, 0]
    nodes_height[:] = [real_height + 0.2, real_height + 0.1, real_height - 0.2]
    update_alfas_precalc()
    res_12 = distance_12()
    res_23 = distance_23()
    res_13 = distance_13()

    print "{}, {}, {}".format(res_12, res_23, res_13)

