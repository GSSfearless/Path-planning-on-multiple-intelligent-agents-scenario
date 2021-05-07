"""

Potential Field based path planner

author: Atsushi Sakai (@Atsushi_twi)

Ref:
https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf

"""

from collections import deque
import numpy as np
import matplotlib.pyplot as plt

# Parameters
KP = 3.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain

AREA_WIDTH = 10.0  # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 3

show_animation = True


def calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy):
    minx = min(min(ox), sx, gx) - AREA_WIDTH / 2.0
    miny = min(min(oy), sy, gy) - AREA_WIDTH / 2.0
    maxx = max(max(ox), sx, gx) + AREA_WIDTH / 2.0
    maxy = max(max(oy), sy, gy) + AREA_WIDTH / 2.0
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, ox, oy, rr)
            uf = ug + uo
            pmap[ix][iy] = uf

    return pmap, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)



def calc_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0


def get_motion_model():
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion


def oscillations_detection(previous_ids, ix, iy):
    previous_ids.append((ix, iy))

    if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
        previous_ids.popleft()

    # check if contains any duplicates by copying into a set
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False


def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr, sx, sy)

    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)

    if show_animation:
        draw_heatmap(pmap)
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(ix, iy, "*k")
        plt.plot(gix, giy, "*m")

    rx, ry = [sx], [sy]
    motion = get_motion_model()
    previous_ids = deque()

    while d >= reso:
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                p = float("inf")  # outside area
                print("outside potential!")
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny
        d = np.hypot(gx - xp, gy - yp)
        rx.append(xp)
        ry.append(yp)

        if (oscillations_detection(previous_ids, ix, iy)):
            print("Oscillation detected at ({},{})!".format(ix, iy))
            break

        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.01)

    print("Goal!!")

    return rx, ry


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)


def main():
    print("potential_field_planning start")

    sx = -40 # start x position [m]
    sy = -40  # start y positon [m]
    gx = 40  # goal x position [m]
    gy = 40  # goal y position [m]
    grid_size = 0.5  # potential grid size [m]
    robot_radius = 1.0  # robot radius [m]
    ox, oy = [], []
    for i in range(-40, 20):
        ox.append(70)
        oy.append(i)
    for i in range(0, 70):
        ox.append(i)
        oy.append(-40)
    for i in range(40, 50):
        ox.append(0.0)
        oy.append(0 - i)
    for i in range(-50, 0):
        ox.append(i)
        oy.append(-50)
    for i in range(-50, 20):
        ox.append(-50)
        oy.append(i)
    for i in range(-70, -50):
        ox.append(i)
        oy.append(20)
    for i in range(20, 50):
        ox.append(-70)
        oy.append(i)
    for i in range(-70, 60):
        ox.append(i)
        oy.append(50)
    for i in range(20, 50):
        ox.append(60)
        oy.append(i)
    for i in range(60, 70):
        ox.append(i)
        oy.append(20)
    '''
    # Experiment 01 Start
    # people 1
    for i in range(-26, -24):
        ox.append(i)
        oy.append(16)
    for i in range(14, 16):
        ox.append(-24)
        oy.append(i)
    for i in range(-26, -24):
        ox.append(i)
        oy.append(14)
    for i in range(14, 16):
        ox.append(-26)
        oy.append(i)
    # people 2
    for i in range(14, 16):
        ox.append(i)
        oy.append(16)
    for i in range(14, 16):
        ox.append(16)
        oy.append(i)
    for i in range(14, 16):
        ox.append(i)
        oy.append(14)
    for i in range(14, 16):
        ox.append(14)
        oy.append(i)
    # people 3
    for i in range(-36, -34):
        ox.append(i)
        oy.append(-29)
    for i in range(-31, -29):
        ox.append(-34)
        oy.append(i)
    for i in range(-36, -34):
        ox.append(i)
        oy.append(-31)
    for i in range(-31, -29):
        ox.append(-36)
        oy.append(i)
    # people 4
    for i in range(19, 21):
        ox.append(i)
        oy.append(-29)
    for i in range(-31, -29):
        ox.append(21)
        oy.append(i)
    for i in range(19, 21):
        ox.append(i)
        oy.append(-31)
    for i in range(-31, -29):
        ox.append(19)
        oy.append(i)
    # setting 20x20 obstacle
    for i in range(-10, 10):
        ox.append(10)
        oy.append(i)
    for i in range(-10, 10):
        ox.append(i)
        oy.append(-10)
    for i in range(-10, 10):
        ox.append(-10)
        oy.append(i)
    for i in range(-10, 10):
        ox.append(i)
        oy.append(10)
    # Experiment 01 End
    
    '''
    # Experiment 02 Start
    for i in range(-10, 10):
        ox.append(10)
        oy.append(i)
    for i in range(-10, 10):
        ox.append(i)
        oy.append(-10)
    for i in range(-10, 10):
        ox.append(-10)
        oy.append(i)
    for i in range(-10, 10):
        ox.append(i)
        oy.append(10)

    for i in range(15,35):
        ox.append(45)
        oy.append(i)
    for i in range(25,45):
        ox.append(i)
        oy.append(15)
    for i in range(15,35):
        ox.append(25)
        oy.append(i)
    for i in range(25,45):
        ox.append(i)
        oy.append(35)

    for i in range(-35,-15):
        ox.append(45)
        oy.append(i)
    for i in range(25,45):
        ox.append(i)
        oy.append(-35)
    for i in range(-35,-15):
        ox.append(25)
        oy.append(i)
    for i in range(25,45):
        ox.append(i)
        oy.append(-15)

    for i in range(-35,-15):
        ox.append(-25)
        oy.append(i)
    for i in range(-45,-25):
        ox.append(i)
        oy.append(-35)
    for i in range(-35,-15):
        ox.append(-45)
        oy.append(i)
    for i in range(-45,-25):
        ox.append(i)
        oy.append(-15)

    for i in range(15,35):
        ox.append(-25)
        oy.append(i)
    for i in range(-45,-25):
        ox.append(i)
        oy.append(15)
    for i in range(15,35):
        ox.append(-45)
        oy.append(i)
    for i in range(-45,-25):
        ox.append(i)
        oy.append(35)

    # Experiment 02 End
    '''
    # Experiment 03 Start
    for i in range(-10, 10):
        ox.append(10)
        oy.append(i)
    for i in range(-10, 10):
        ox.append(i)
        oy.append(-10)
    for i in range(-10, 10):
        ox.append(-10)
        oy.append(i)
    for i in range(-10, 10):
        ox.append(i)
        oy.append(10)

    for i in range(15,35):
        ox.append(45)
        oy.append(i)
    for i in range(25,45):
        ox.append(i)
        oy.append(15)
    for i in range(15,35):
        ox.append(25)
        oy.append(i)
    for i in range(25,45):
        ox.append(i)
        oy.append(35)

    for i in range(-35,-15):
        ox.append(45)
        oy.append(i)
    for i in range(25,45):
        ox.append(i)
        oy.append(-35)
    for i in range(-35,-15):
        ox.append(25)
        oy.append(i)
    for i in range(25,45):
        ox.append(i)
        oy.append(-15)

    for i in range(-35,-15):
        ox.append(-25)
        oy.append(i)
    for i in range(-45,-25):
        ox.append(i)
        oy.append(-35)
    for i in range(-35,-15):
        ox.append(-45)
        oy.append(i)
    for i in range(-45,-25):
        ox.append(i)
        oy.append(-15)

    for i in range(15,35):
        ox.append(-25)
        oy.append(i)
    for i in range(-45,-25):
        ox.append(i)
        oy.append(15)
    for i in range(15,35):
        ox.append(-45)
        oy.append(i)
    for i in range(-45,-25):
        ox.append(i)
        oy.append(35)

    for i in range(-10,10):
        ox.append(45)
        oy.append(i)
    for i in range(25,45):
        ox.append(i)
        oy.append(-10)
    for i in range(-10,10):
        ox.append(25)
        oy.append(i)
    for i in range(25,45):
        ox.append(i)
        oy.append(10)

    for i in range(15,35):
        ox.append(10)
        oy.append(i)
    for i in range(-10,10):
        ox.append(i)
        oy.append(15)
    for i in range(15,35):
        ox.append(-10)
        oy.append(i)
    for i in range(-10,10):
        ox.append(i)
        oy.append(35)

    for i in range(-35,-15):
        ox.append(10)
        oy.append(i)
    for i in range(-10,10):
        ox.append(i)
        oy.append(-35)
    for i in range(-35,-15):
        ox.append(-10)
        oy.append(i)
    for i in range(-10,10):
        ox.append(i)
        oy.append(-15)

    for i in range(-10,10):
        ox.append(-25)
        oy.append(i)
    for i in range(-45,-25):
        ox.append(i)
        oy.append(-10)
    for i in range(-10,10):
        ox.append(-45)
        oy.append(i)
    for i in range(-45,-25):
        ox.append(i)
        oy.append(10)

    # Experiment 03 End
    
    
    # Experiment 04/05  Start

    d = 4
    l = 30
    for i in range(10 - l, 10):
        ox.append(70 - d)
        oy.append(i)
    for i in range(-50 + d, 70 - d):
        ox.append(i)
        oy.append(10 - l)
    for i in range(10 - l, 10):
        ox.append(-50 + d)
        oy.append(i)
    for i in range(-50 + d, 70 - d):
        ox.append(i)
        oy.append(10)

    # Experiment 04/05 End
    '''

    if show_animation:
        plt.grid(True)
        plt.axis("equal")

    # path generation
    _, _ = potential_field_planning(
        sx, sy, gx, gy, ox, oy, grid_size, robot_radius)

    if show_animation:
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
