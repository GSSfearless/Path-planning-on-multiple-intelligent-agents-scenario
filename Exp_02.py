# -*- coding: utf-8 -*-
"""
Created on Wed Jan 20 17:44:30 2021

@author: GSS-fearless
"""

import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from matplotlib.lines import Line2D
pos = []  # position list
vel = []  # velocity list
abs_v1 = []
abs_v2 = []
abs_v3 = []
abs_v4 = []
abs_v5 = []
total_time = []
T_distance = []
rx, ry = [], []

v_max = 10  # maximum speed

# For attractive parameters
k1 = 0.5
k2 = 40
k3 = 150

ln = 1  # ob_radius
m = [2, 2, 2, 2, 2]  # mass
# parameters
i0 = 10  # obstacle_stimuli
a = 15
b = 5
y = 10
# boundary_stimuli
i0b = 250
lnb = 1
ab = 150
yb = 1000000
# boundary location
line1 = [(70, 20), (70, -40)]
line2 = [(70, -40), (0, -40)]
line3 = [(0, -40), (0, -50)]
line4 = [(0, -50), (-50, -50)]
line5 = [(-50, -50),(-50, 20)]
line6 = [(-50, 20),(-70, 20)]
line7 = [(-70,20),(-70, 50)]
line8 = [(-70,50),(60, 50)]
line9 = [(60,50),(60, 20)]
line10 = [(60,20),(70, 20)]
# Rectangle
line11 = [(10,10),(10,-10)]
line12 = [(10,-10),(-10,-10)]
line13 = [(-10,-10),(-10,10)]
line14 = [(-10,10),(10,10)]
#Exp_02
line15 = [(45,35),(45,15)]
line16 = [(45,15),(25,15)]
line17 = [(25,15),(25,35)]
line18 = [(25,35),(45,35)]

line19 = [(-25,35),(-25,15)]
line20 = [(-25,15),(-45,15)]
line21 = [(-45,15),(-45,35)]
line22 = [(-45,35),(-25,35)]

line23 = [(-25,-15),(-25,-35)]
line24 = [(-25,-35),(-45,-35)]
line25 = [(-45,-35),(-45,-15)]
line26 = [(-45,-15),(-25,-15)]

line27 = [(45,-15),(45,-35)]
line28 = [(45,-35),(25,-35)]
line29 = [(25,-35),(25,-15)]
line30 = [(25,-15),(45,-15)]

line = [line1, line2, line3, line4, line5, line6, line7, line8, line9, line10,line11,line12,line13,line14,line15, line16, line17, line18,line19,line20,line21,line22,
        line23, line24, line25, line26, line27, line28, line29, line30]  # force given in clockwise
#line_direction = [(-1,0),(0,-1),(-1,0),(0,-1),(1,0),(0,1),(1,0),(0,-1),(-1,0),(0,-1),(1,0),(0,1),(-1,0),(0,1)]
line_direction = [(1,0),(0,-1),(1,0),(0,-1),(-1,0),(0,-1),(-1,0),(0,1),(1,0),(0,1),(-1,0),(0,1),(1,0),(0,-1),(-1,0),(0,1),(1,0),(0,-1),(-1,0),(0,1),(1,0),(0,-1),(-1,0),(0,1),(1,0),(0,-1),(-1,0),(0,1),(1,0),(0,-1)]
pos.append([np.array([-40, -40])])  # initial position
vel.append([np.array([1, 0])])  # init speed = current direction, body
body = Circle(xy=(-40, -40), radius=1, fc='r', ec='r', alpha=1)
# object number M objects + point of interactions
ob = [body]  # object list
#print('ob',ob)
obr = 1.5     # relates to the target destination
tar = []
target_n = 1  # target position
v = 4 * np.random.random_sample((2,))
vel.append([v])
gx = 40
gy = 40
sx = -40
sy = -40
t1 = [gx,gy]
target = Circle(xy=t1, radius=obr, fc='g', ec='g', alpha=0.15)
tar.append(target)


t = [time.time()]                                   # time.time() will return the timestamp
tc = [0]  # sampling period
def count_time():
    t.append(time.time())
    tb = (t[-1] - t[-2])
    tc.append(tb)
    '''print('here is t:',t)
    print('here is tb:',tb)
    print('here is tc',tc)'''
def delta_v(j, i): # trend of collision
    dr = get_direction(j, i)
    dd = 1
    #print('vel[j]',vel[j])
    vj = (vel[j][-1]) ** 2
    vj = (vj[0] + vj[1]) ** 0.5
    vjr = vel[j][-1]
    vi = (vel[i][-1]) ** 2
    vi = (vi[0] + vi[1]) ** 0.5
    vir = vel[i][-1]
    #print('vel[j][-1]',vel[j][-1])
    '''print('dr:',dr)
    print('vj:',vj)
    print('vjr:',vjr)
    print('vi:',vi)
    print('vir:',vir)'''
    cos_j = np.dot(dr, vjr) / (vj * dd)
    cos_i = np.dot(dr, vir) / (vi * dd)
    #print('this is cos_j',cos_j)
    if cos_j < 0:
        vjr = -vjr
    if cos_i > 0:
        vir = -vir
    va = vjr + vir
    return va
def delta_vb(j, i):
    dr = np.array(line_direction[i])
    dd = 1
    vj = (vel[j][-1]) ** 2
    vj = (vj[0] + vj[1]) ** 0.5
    vjr = vel[j][-1]
    cos_j = np.dot(dr, vjr) / (vj * dd)
    if cos_j < 0:
        vjr = -vjr
    va = vjr
    return va

def t_distance(j):
    # distance to the target
    distance = np.array(tar[j].get_center()) - np.array(ob[j].get_center())
    distance = distance ** 2
    distance = (distance[0] + distance[1]) ** 0.5
    #print('distance',distance)
    return distance  # always positive

def t_direction(j):
    direction = np.array(tar[j].get_center()) - np.array(ob[j].get_center())
    distance = direction ** 2
    distance = (distance[0] + distance[1]) ** 0.5
    direction = direction / (distance + 1)
    #print('direction', direction)
    return direction


def foot_point(j, i):

    """
    # Perpendicular foot from point_j to line_i (boundary)
    x0 = pos[j][-1][0]
    y0 = pos[j][-1][1]
    x1 = line[i][0][0]
    y1 = line[i][0][1]
    x2 = line[i][1][0]
    y2 = line[i][1][1]
    k = -((x1 - x0) * (x2 - x1) + (y1 - y0) * (y2 - y1)) / ((x2 - x1) ** 2 + (y2 - y1) ** 2)
    xn = k * (x2 - x1) + x1
    yn = k * (y2 - y1) + y1
    point = np.array([xn,yn])
    return point
    """

    x = 0
    y = 0
    x0 = pos[j][-1][0]
    y0 = pos[j][-1][1]
    x1 = line[i][0][0]
    y1 = line[i][0][1]
    x2 = line[i][1][0]
    y2 = line[i][1][1]
    if y1 == y2:
        if x2 > x1:
            if (x0 > x1) and (x0 < x2):
                x = x0
                y = y1
            elif x0 > x2:
                x = x2
                y = y1
            elif x0 < x1:
                x = x1
                y = y1
        if x1 > x2:
            if (x0 > x2) and (x0 < x1):
                x = x0
                y = y1
            elif x0 > x1:
                x = x1
                y = y1
            elif x0 < x2:
                x = x2
                y = y1
        point = np.array([x, y])      # output [x0,y1] when y1==y2
    else:
        if x1 == x2:
            if y2 > y1:
                if (y0 > y1) and (y0 < y2):
                    x = x1
                    y = y0
                elif y0 > y2:
                    x = x1
                    y = y2
                elif y0 < y1:
                    x = x1
                    y = y1
            if y1 > y2:
                if (y0 > y2) and (y0 < y1):
                    x = x1
                    y = y0
                elif y0 > y1:
                    x = x1
                    y = y1
                elif y0 < y2:
                    x = x1
                    y = y2
            point = np.array([x, y])    # output [x1,y0] when x1==x2
        else:
            point = np.array([0,0])
    return point



def siep(j, i):
    tp = tc[-1]
    si = i0 + (a * ln + b * vel[i][-1] * tp) / (1 + np.exp(delta_v(j, i)) / y)
    delta_p = get_distance(j, i)
    f = si / delta_p
    return f
    # return force(scalar)

'''
def f_boundary(j,i):
    distance = (foot_point(j,i) - pos[j][-1]) ** 2
    f = i0b[i] + ab[i]/(distance + 1)
    print('distance',distance)
    if distance.any() < 200:
        f = f * 2
    else:
        if distance.all() > 1000:
            f = 0
    print('f_boundary',f)
    return f
'''



def siep_boundary(j, i):
    si = i0b + (ab * lnb) / (1 + np.exp(delta_vb(j, i)) / yb)
    distance = (foot_point(j, i) - pos[j][-1]) ** 2
    delta_p = (distance[0] + distance[1]) ** 0.5
    f = si / (delta_p + 1)

    '''
    if (distance[0] > 800) and (distance[1] > 800):
        f = 0
    else:
        if (distance[0] < 200) or (distance[1] < 200):
            f = 1.5 * f
    '''
    return f  #  scalar



def controller(j):
    # stop by the target
    begin_time = time.time()
    td = t_distance(j)
    if td < 1:
        tp = np.array(tar[j].get_center())
        return tp
        # stimuli-induced force from ob
    f = np.array([0, 0])
        # stimuli-induced force from boundary
    for i in range(len(line)):
        f = f + np.array(line_direction[i]) * siep_boundary(j, i)
    # target attraction
    T_distance.append(t_distance(j))
    f_target = t_direction(j) * (k1 * t_distance(j) + k2 + k3/t_distance(j))
    f = f_target - f
    # acceleration
    acc = f / m[j]
    #print('acc',acc)
    tp = tc[-1]
    vr = vel[j][-1] + acc * tp
    vm = vr ** 2
    vm = (vm[0] + vm[1]) ** 0.5
    # speed limitation
    if vm <= v_max:
        vn = vr
    else:
        vn = vr / vm * v_max
    #print('vn:',vn)
    vel[j].append(vn)
    p = pos[j][-1] + vn * tp
    #print('this is pos[j][-1]',pos[j][-1])
    #print('this is pos[j][-1][0]',pos[j][-1][0])
    rx.append(p[0])
    ry.append(p[1])
    pos[j].append(p)
    abs_1 = (vel[0][-1][0]**2 + vel[0][-1][1]**2)**0.5
    abs_v1.append(abs_1)

    end_time = time.time()
    run_time = end_time - begin_time
    total_time.append(run_time)
    return p               # return current position

def animate():
    fig, ax1 = plt.subplots(1, 1, figsize=(12, 9))
    def init():
        ax1.set_xlim([-80, 80])
        ax1.set_ylim([-60, 60])
        ax1.set_title("SIEP control scenario", size=20)
        # add lines
        (line1_xs, line1_ys) = zip(*line1)       # zip(*) means unzip
        (line2_xs, line2_ys) = zip(*line2)
        (line3_xs, line3_ys) = zip(*line3)
        (line4_xs, line4_ys) = zip(*line4)
        (line5_xs, line5_ys) = zip(*line5)       # zip(*) means unzip
        (line6_xs, line6_ys) = zip(*line6)
        (line7_xs, line7_ys) = zip(*line7)
        (line8_xs, line8_ys) = zip(*line8)
        (line9_xs, line9_ys) = zip(*line9)
        (line10_xs, line10_ys) = zip(*line10)
        (line11_xs, line11_ys) = zip(*line11)
        (line12_xs, line12_ys) = zip(*line12)
        (line13_xs, line13_ys) = zip(*line13)
        (line14_xs, line14_ys) = zip(*line14)
        (line15_xs, line15_ys) = zip(*line15)
        (line16_xs, line16_ys) = zip(*line16)
        (line17_xs, line17_ys) = zip(*line17)
        (line18_xs, line18_ys) = zip(*line18)
        (line19_xs, line19_ys) = zip(*line19)
        (line20_xs, line20_ys) = zip(*line20)
        (line21_xs, line21_ys) = zip(*line21)
        (line22_xs, line22_ys) = zip(*line22)
        (line23_xs, line23_ys) = zip(*line23)
        (line24_xs, line24_ys) = zip(*line24)
        (line25_xs, line25_ys) = zip(*line25)
        (line26_xs, line26_ys) = zip(*line26)
        (line27_xs, line27_ys) = zip(*line27)
        (line28_xs, line28_ys) = zip(*line28)
        (line29_xs, line29_ys) = zip(*line29)
        (line30_xs, line30_ys) = zip(*line30)

        '''print('line1_xs',line1_xs)
        print('line1_ys',line1_ys)'''
        lw = 2
        ax1.add_line(Line2D(line1_xs, line1_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line2_xs, line2_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line3_xs, line3_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line4_xs, line4_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line5_xs, line5_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line6_xs, line6_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line7_xs, line7_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line8_xs, line8_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line9_xs, line9_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line10_xs, line10_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line11_xs, line11_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line12_xs, line12_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line13_xs, line13_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line14_xs, line14_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line15_xs, line15_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line16_xs, line16_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line17_xs, line17_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line18_xs, line18_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line19_xs, line19_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line20_xs, line20_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line21_xs, line21_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line22_xs, line22_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line23_xs, line23_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line24_xs, line24_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line25_xs, line25_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line26_xs, line26_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line27_xs, line27_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line28_xs, line28_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line29_xs, line29_ys, linewidth=lw, color='g'))
        ax1.add_line(Line2D(line30_xs, line30_ys, linewidth=lw, color='g'))
        for i in range(len(ob)):
            ax1.add_artist(ob[i])
        for j in range(len(tar)):
            ax1.add_artist(tar[j])
    def draw(n):
    # sampling
        count_time()
        for j in range(len(ob)):
            npo = controller(j)
            ob[j].set_center(npo)
            #print('This is ob[j]',ob[j])
    ani = FuncAnimation(fig, draw, init_func=init, frames=1000, interval=15, blit=False)
    #ani.save('/home/ryan/SIEP_control.gif',writer='pillow',fps=30)
    plt.show()
    T1 = np.linspace(1, len(abs_v1) * 100, len(abs_v1))
    plt.scatter(T1,abs_v1,s=5)
    plt.title('Robot velocity')
    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.show()
    T2 = np.linspace(1,len(T_distance)*100,len(T_distance))
    plt.scatter(T2,T_distance,color='orange',s=5)
    plt.title('Robot-target distance')
    plt.xlabel('Time')
    plt.ylabel('Distance')
    plt.show()


def show():
    fig, ax1 = plt.subplots(1, 1, figsize=(12, 9))
    ax1.set_xlim([-80, 80])
    ax1.set_ylim([-60, 60])
    ax1.set_title("SIEP control scenario", size=20)
    # add lines
    (line1_xs, line1_ys) = zip(*line1)  # zip(*) means unzip
    (line2_xs, line2_ys) = zip(*line2)
    (line3_xs, line3_ys) = zip(*line3)
    (line4_xs, line4_ys) = zip(*line4)
    (line5_xs, line5_ys) = zip(*line5)  # zip(*) means unzip
    (line6_xs, line6_ys) = zip(*line6)
    (line7_xs, line7_ys) = zip(*line7)
    (line8_xs, line8_ys) = zip(*line8)
    (line9_xs, line9_ys) = zip(*line9)
    (line10_xs, line10_ys) = zip(*line10)
    (line11_xs, line11_ys) = zip(*line11)
    (line12_xs, line12_ys) = zip(*line12)
    (line13_xs, line13_ys) = zip(*line13)
    (line14_xs, line14_ys) = zip(*line14)
    (line15_xs, line15_ys) = zip(*line15)
    (line16_xs, line16_ys) = zip(*line16)
    (line17_xs, line17_ys) = zip(*line17)
    (line18_xs, line18_ys) = zip(*line18)
    (line19_xs, line19_ys) = zip(*line19)
    (line20_xs, line20_ys) = zip(*line20)
    (line21_xs, line21_ys) = zip(*line21)
    (line22_xs, line22_ys) = zip(*line22)
    (line23_xs, line23_ys) = zip(*line23)
    (line24_xs, line24_ys) = zip(*line24)
    (line25_xs, line25_ys) = zip(*line25)
    (line26_xs, line26_ys) = zip(*line26)
    (line27_xs, line27_ys) = zip(*line27)
    (line28_xs, line28_ys) = zip(*line28)
    (line29_xs, line29_ys) = zip(*line29)
    (line30_xs, line30_ys) = zip(*line30)

    lw = 2
    ax1.add_line(Line2D(line1_xs, line1_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line2_xs, line2_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line3_xs, line3_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line4_xs, line4_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line5_xs, line5_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line6_xs, line6_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line7_xs, line7_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line8_xs, line8_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line9_xs, line9_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line10_xs, line10_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line11_xs, line11_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line12_xs, line12_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line13_xs, line13_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line14_xs, line14_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line15_xs, line15_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line16_xs, line16_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line17_xs, line17_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line18_xs, line18_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line19_xs, line19_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line20_xs, line20_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line21_xs, line21_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line22_xs, line22_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line23_xs, line23_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line24_xs, line24_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line25_xs, line25_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line26_xs, line26_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line27_xs, line27_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line28_xs, line28_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line29_xs, line29_ys, linewidth=lw, color='g'))
    ax1.add_line(Line2D(line30_xs, line30_ys, linewidth=lw, color='g'))

    plt.plot(gx,gy,'xb')
    plt.plot(sx,sy,'og')
    plt.plot(rx,ry,'-r')

    plt.show()

animate()
show()
print('length:',len(rx))