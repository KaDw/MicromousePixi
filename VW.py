# -*- coding: utf-8 -*-
# To change this license header, choose License Headers in Project Properties.
# To change this template file, choose Tools | Templates
# and open the template in the editor.

import math
import matplotlib.pyplot as pl
import numpy as np
import scipy.signal as sp

MOTOR_DRIVER_FREQ = 400
MOTOR_DRIVER_T = 1/MOTOR_DRIVER_FREQ
MOTOR_ACC_V = 1000
MOTOR_ACC_W = 200
HALF_WHEELBASE = 33

tv = 0
tw = 0
motors = {''}
targetW =0


def MotorFindParams(previousV, vel, dist, acc, time):
    dV = vel - previousV;
    Tacc = dV / acc;
    Sbreak = 0.5 * vel * vel / acc;
    Sacc = (previousV * Tacc + 0.5 * dV * dV / acc);
    if (Sacc + Sbreak > dist):
        vel = math.sqrt(acc*dist + 0.5*previousV*previousV)
        dV = vel - previousV;
        Tacc = dV / acc;
        Sacc = (0.5 * (previousV * Tacc + dV * dV / acc));
        Sbreak = 0.5 * vel * vel / acc;
        print("Droga była za duża, nowe V:%f" % vel)
    time = (Tacc + (dist - Sacc - Sbreak) / vel)# *MOTOR_DRIVER_FREQ;
    print("droga przysp: %d." % Sacc)
    print("droga hamowania: %d." % Sbreak)
    print("w sumie: %d." % (Sbreak+Sacc))
    print("dystans %d" % dist)
    return time, vel

def MotorGoA(left, right, vel):
    tv=0
    tw=0
    velv = 0
    velw = 0
    previousV = 0;
    previousW = 0;
    tv, velv = MotorFindParams(previousV, vel, 0.5*(math.fabs(left) + math.fabs(right)), MOTOR_ACC_V, tv);
    print("Prędkość końcowa:", velv)
    print(" czas:", tv)
    print(" T: ", round(tv*MOTOR_DRIVER_FREQ))
    if (left != right):
        alpha = 0.5*(left - right) / HALF_WHEELBASE;
        w = alpha * vel / (abs(left) + abs(right)); #w = alpha/t = alpha*v/s
        #przyspieszenie odśrodkowe
        R = (left+right)/(left-right)*HALF_WHEELBASE
        if(w*w*R > MOTOR_ACC_W):
            w = math.sqrt(MOTOR_ACC_W/R)
        tw, velw = MotorFindParams(0, w, alpha, MOTOR_ACC_W, tw);
        if tv < tw and 0:
            print("czekam na obrot")
            a = 1/MOTOR_ACC_V
            b = -2*previousV/MOTOR_ACC_V - (tw + w/MOTOR_ACC_W)
            c = 0.5*(left+right) + 1.5*previousV*previousV/MOTOR_ACC_V
            sdelta = math.sqrt(b*b - 4*a*c)
            newvel = 0.5*(b-sdelta)/a
            #Tw = vel/MOTOR_ACC_V - 2*previousV/MOTOR_ACC_V + (0.5*(left+right)+3*previousV*previousV/2/MOTOR_ACC_V)/vel
            if(not(0 < newvel and newvel < vel)):
                newvel = 0.5*(b+sdelta)/a
            velv = newvel
        elif tw < tv and 0:
            print("czekam na translacje")
            a = 1/MOTOR_ACC_W
            b = -2*previousW/MOTOR_ACC_W - (tv + vel/MOTOR_ACC_V)
            c = alpha + previousW*previousW/MOTOR_ACC_W
            #c = 0.5*(left+right) + 1.5*previousW*previousW/MOTOR_ACC_W #todo: moze abs na left i right?
            sdelta = math.sqrt(b*b - 4*a*c)
            neww = 0.5*(b-sdelta)/a
            if(not(0 < neww and neww < velw)):
                neww = 0.5*(b+sdelta)/a
            velw = neww
            #velw = alpha/tv
    else:
        velw = 0;

    if (left + right >= 0):
        previousV = velv;
        targetV = 2 * velv;
    else:
        previousV = -velv;
        targetV = -2 * velv;
    return tv, velv, velw, alpha


def MotorTurnA(alpha, R, vel):
    previousV = 0
    previousW = 0
    tv = 0
    tw = 0
    velv = velw = 0
    dist = math.pi*R*alpha/180
    #w = math.sqrt(MOTOR_ACC_W/R) #przysp dosrodkowe
    w = vel/(abs(R)+HALF_WHEELBASE);
    if(dist != 0):
        tv, velv = MotorFindParams(previousV, vel, dist, MOTOR_ACC_V, tv);
    if(alpha != 0):
        tw, velw = MotorFindParams(previousW, w, alpha, MOTOR_ACC_W, tw);
    return tv, velv, velw, alpha, dist

vk = 2011
vw = 0
dist = 100

timv, velv, velw, alpha, dist = MotorTurnA(180*math.pi/180, 30, vk)

currV = 0
t = [0]
y = [dist]
a = [alpha]
velv_mem=[0]
velw_mem=[0]


Sbrv = velv*velv/MOTOR_ACC_V*0.5
Sbrw = velw*velw/MOTOR_ACC_W*0.3

print("velv: %d" % velv)
print("velw: %d" % velw)
print("Sbrw: %f" % Sbrw)
for i in range(1, round(50 + (timv + velw/MOTOR_ACC_W)*MOTOR_DRIVER_FREQ)):
    t.append(i)
    velv_mem.append(velv_mem[i-1])
    velw_mem.append(velw_mem[i-1])
    #tworzenie trapeza z predkosci
    #na podstawie velv i velw
    #velocity
    if velv_mem[i] < velv:
        velv_mem[i] += MOTOR_DRIVER_T*MOTOR_ACC_V
    if velv < velv_mem[i]:
        velv_mem[i] -= MOTOR_DRIVER_T*MOTOR_ACC_V
    #rotation
    if velw_mem[i] < velw:
        velw_mem[i] += MOTOR_DRIVER_T*MOTOR_ACC_W
        if velw_mem[i] > velw:
            velw_mem[i] = velw
    if velw < velw_mem[i]:
        velw_mem[i] -= MOTOR_DRIVER_T*MOTOR_ACC_W
        if velw_mem[i] < velw:
            velw_mem[i] = velw
    #break
    #porownujemy droge chamowania z kierunkiem obrotu kol
    #przy porownywaniu z pozostala droga moga wystepowac bledy!
    if velv > 0 and dist - Sbrv <= 0:
        velv = 0
    elif velv < 0 and dist - Sbrv >= 0:
        velv = 0
    if velw > 0 and alpha - Sbrw <= 0:
        velw = 0
    elif velw < 0 and alpha - Sbrw >= 0:
        velw = 0;
    #zmniejsz droge do pokonania
    dist -= velv_mem[i]*MOTOR_DRIVER_T
    alpha-= velw_mem[i]*MOTOR_DRIVER_T
    y.append(dist)
    a.append(alpha)

print("Minimal value of track %d" % min(y))
print("Minimal value of angle %f" % min(a))
pl.figure(1)
pl.subplot(411)
pl.plot(t, y)
pl.ylabel('[mm]')
pl.subplot(412)
pl.plot(t, velv_mem)
pl.ylabel("[mm/s]")
pl.subplot(413)
pl.plot(t, a)
pl.ylabel('[rad]')
pl.subplot(414)
pl.plot(t, velw_mem)
pl.ylabel("[rad/s]")
pl.xlabel("[T]")
pl.show()


#sys = sp.ss2tf(A, B, C, D)
#w, H = sp.freqresp(sys)
#pl.figure()
#pl.plot(H.real, H.imag, "b")
#pl.plot(H.real, -H.imag, "r")
#pl.show()