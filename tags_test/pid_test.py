# coding:utf-8
import numpy as np
from matplotlib import pyplot as plt
from numpy.random import *

def main():
    M = 1.00
    M1 =  0.00

    e = 0.00
    e1 = 0.00
    e2 = 0.00

    Kp = 0.20
    Ki = 0.0
    Kd = 0.0

    t = 100

    goal = 100.00

    x_list = []
    y_list = []

    x_list.append(0)
    y_list.append(0.00)

    for i in range(1,t):
        M1 = M
        e2 = e1
        e1 = e
        e = goal - y_list[i-1]

        M = M1 + Kp * (e-e1) + Ki * e + Kd * ((e-e1) - (e1-e2))

        y_list.append(M)
        x_list.append(i)

    plt.plot(x_list, y_list)
    plt.ylim(0, goal*2)
    plt.show()
 
    print (y_list())

if __name__ == "__main__":
    main()