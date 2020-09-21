#!/usr/bin/python

import numpy as np

class Model_robot:

    def __init__(self):

        # Parametros de las ruedas
        radius = 0.06 # m

        # Para rueda frontal

        alpha_f = 0.0   # radians
        beta_f  = 0.0   # radians
        l_f     = 0.433 # m

        # Para rueda izquierda

        alpha_l = np.pi / 2
        beta_l  = 0.0
        l_l     = 0.25

        # Para rueda derecha

        alpha_r = - np.pi /2
        beta_r  =   np.pi
        l_r     = 0.25

        # Definicion matrices J1 y J2

        J1 = np.array(  [ (np.sin(alpha_f+beta_f),  -np.cos(alpha_f+beta_f), -l_f*np.cos(beta_f) ),
                          (np.sin(alpha_l+beta_l),  -np.cos(alpha_l+beta_l), -l_l*np.cos(beta_l) ),
                          (np.sin(alpha_r+beta_r),  -np.cos(alpha_r+beta_r), -l_r*np.cos(beta_r) ) ])

        J2 = radius*np.identity(3)

        # Definir Jacob_inv

        self.Jacob_inv = np.matmul ( np.linalg.pinv(J2) , J1 )

    #----------------------------------------------------------------------------------------------#
    #----------------------------------------------------------------------------------------------#
    def calcVelWheels (self, vel_x, vel_y, vel_ang):

        vec_Vel = np.array([(vel_x),(vel_y),(vel_ang)]) # Velocidad en el marco del robot

        velWheels = np.matmul(self.Jacob_inv, vec_Vel)

        return velWheels