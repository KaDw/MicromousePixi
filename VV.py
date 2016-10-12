# -*- coding: utf-8 -*-

import math
import matplotlib.pyplot as pl

MOTOR_ACC_V = 1200
MOTOR_DRIVER_T = 0.001

class motors_t:
    def __init__(self):
        self.mot0 = 0 #dest vel float
        self.mot1 = 0
        self.cmot0 = 0 #current vel float
        self.cmot1 = 0
        self.time = 0
        self.previousV = 0.0


def MotorCalcTime(lastV, s, vel): #float float float
    Tacc = abs(vel-lastV)/MOTOR_ACC_V #float, use fabs()
    Sacc = (lastV+vel)*0.5*Tacc #float
    Tcon = abs((s-Sacc)/vel) #float
    T = 0 #float
    if(abs(Sacc) > abs(s)): #fabs
        # gdy tylko przyspieszamy, to czas jest opisany zaleznoscia
        # t^2 + t*2*v0/a - s*s/a = 0
        print("Time - part")
        s = abs(s) #fabs
        T = (-lastV + math.sqrt(lastV*lastV+2*s*MOTOR_ACC_V))/MOTOR_ACC_V #liczba pod pierwistkiem jest dodatnia
    elif(vel != 0 ):
        # gdy przyspieszamy oraz poruszamy sie ze stala predkoscia
        # T = Tacc + Tconst
        print("Time - full")
        T = Tacc + Tcon
    return T


def MotorCalcS(lastV, vel, t): #float float float
    Tacc = abs((vel-lastV)/MOTOR_ACC_V) #float
    Sacc = (lastV+vel)*0.5*Tacc #float
    Scon = vel*(t-Tacc) #float
    if(t > Tacc):
        return Sacc+Scon # float
    else:
        return Sacc # float


def MotorCalcVel(lastV, s, t): #float float float
    #ale gdy mamy rownierz poruszanie sie z predkoscia stala,
    #to musimy rozwiazac ponizsze rownanie z zalozeniami:
    a = MOTOR_ACC_V #float
    if(lastV*t > s):
        a = -a
    #zakladamy, ze v > lastV
    p = (a*t*t + 2*lastV*t - 2*s)/a
    S = 0
    if(p>=0):
        p = math.sqrt(p)
        V = lastV - a*p + a*t
        S = MotorCalcS(lastV, V, t)
        if(abs(S - s) > 5):
            print("Vel - minus ")
            V = lastV + a * p + a * t
    else: # w akcie rozpaczy (gdy nie idzie liczenie tego) - to uprosc
        #obliczenia dobrane metoda doswiadczalna
        #inne opcje to:
        # a) V = s/t //gdy t!=0
        # b) p = math.sqrt(a*t*t + 2*lastV*t + 2*s)/a)
        #    V = lastV + a * p + a * t
        # c) wesola tworczosc
        print("Vel - rozpacz1, t=", t)
        V = lastV + a*t
    return V;


motors = motors_t();
previousV = 10.0


def MotorGoA(left, right, vel):
    lVl = motors.cmot0 #last vel
    lVr = motors.cmot1
    vel = abs(vel)
    Vl = vel
    Vr = vel
    T = 0.0
    if(left < 0):
        Vl = -vel;
    if(right < 0):
        Vr = - vel
    Tl = MotorCalcTime(lVl, left,  Vl)# last vel from left motor
    print("Tl ", Tl)
    Tr = MotorCalcTime(lVr, right, Vr)# last vel from right mototr
    print("Tr: ", Tr)
    if(Tl > Tr):
        T = Tl
    else:
        T = Tr
    Vl = MotorCalcVel(lVl, left,  T)
    Vr = MotorCalcVel(lVr, right, T)
    motors.mot0 = Vl
    motors.mot1 = Vr
    motors.time = T/MOTOR_DRIVER_T
    motors.previousV = vel #zrobic osobno dla kazdego z silnikow


def _stepTo(_from, _to, _step):
    val = _from
    if(_from < _to):
        val = _from + _step
        if(val > _to):
            val = _to
    elif(_from > _to):
        val = _from - _step
        if(val < _to):
            val = _to
    return val


Vl = [motors.previousV]
Vr = [motors.previousV]
Sl = [0]
Sr = [0]
t  = [0]

for i in range(0,5):
    print("\n")
    left = int(input("lewe kolo: "))
    right = int(input("prawe kolo: "))
    vel = int(input("predkosc: "))
    MotorGoA(left, right, vel);
    print("lewy prawy czas")
    print("predkosc: ", motors.mot0, motors.mot1, float(motors.time), sep=', ');
    while(motors.time > 0):
        motors.cmot0 = _stepTo(motors.cmot0, motors.mot0, MOTOR_DRIVER_T*MOTOR_ACC_V)
        motors.cmot1 = _stepTo(motors.cmot1, motors.mot1, MOTOR_DRIVER_T*MOTOR_ACC_V)
        Vl.append(motors.cmot0)
        Vr.append(motors.cmot1)
        Sl.append(Sl[-1]+Vl[-1]*MOTOR_DRIVER_T)
        Sr.append(Sr[-1]+Vr[-1]*MOTOR_DRIVER_T)
        t.append(t[-1]+1)
        motors.time = motors.time - 1
    print("droga na koniec: ", Sl[-1], Sr[-1], sep=', ');
    #wyswietl wyniki
    pl.subplot(2, 1, 1)
    pl.plot(t, Sl)
    pl.plot(t, Sr)
    pl.ylabel('droga')
    pl.subplot(2, 1, 2)
    pl.plot(t, Vl)
    pl.plot(t, Vr)
    pl.ylabel('prędkość')
    pl.show()
