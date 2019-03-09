# -*- coding: utf-8 -*-
"""
Created on Thu Jan 24 17:07:45 2019

@author: Loic Dietz

NOTE : Dernière version mais pas d'interface graphique dans ce code 
"""



### Outil de simulation de trajectoire
## VALROBOTIK - 2018

## Importation des modules utilisés
import tkinter as tk
from tkinter import messagebox
# import time
import numpy as np
import matplotlib.pyplot as plt
import math

####################
#### Simulation ####
####################

# définition des constantes utilisées et normalement récupérées par l'interface graphique
nb_capt = 8
points_robot = [[-6.25, 15.08885], [-15.08885, 6.25], [-15.08885, -6.25], [-6.25, -15.08885], [6.25, -15.08885], [15.08885, -6.25], [15.08885, 6.25], [6.25, 15.08885], [0, 10]]
beta = 5
gamma = 2
alpha  = [['capt0', 10.0], ['capt1', 10.0], ['capt2', 10.0], ['capt3', 10.0], ['capt4', 10.0], ['capt5', 10.0], ['capt6', 10.0], ['capt7', 10.0]]
pos_capteurs = [['capt0', 0.0, 15.08885, 0.0], ['capt1', -10.669425, 10.669425, 45.0], ['capt2', -15.08885, 0.0, 90.0], ['capt3', -10.669425, -10.669425, 135.0], ['capt4', 0.0, -15.08885, 180.0], ['capt5', 10.669425, -10.669425, 225.0], ['capt6', 15.08885, 0.0, 270.0], ['capt7', 10.669425, 10.669425, 315.0]]
Robot = [0, 0, 0]
Objectif = [100, 100]

def angleWrap(a):
    res = a
    if a > math.pi:
        res = a - 2*math.pi
    if a < math.pi:
        res = a + 2*math.pi
    return res        

def zeros(n): # crée une liste contenant n zéros
    L = []
    for i in range(0,n):
        L = L + [0]
        i = i + 1
    return L

def ones(n): # crée une liste contenant n uns
    L = []
    for i in range (0,n):
        L = L + [1]
        i = i + 1
    return L

def list2int(L): # convertit une liste composé d'un seul élément en un entier
    if len(L) == 1:
        return L[0]
    else:
        return 'error'

def change_repere(x, y, x_centre, y_centre, alpha): #permet de passer du repère 1 au repère 2
    #x et y sont des listes contenant les coordonnées dans le repère 1, x_centre et y_centre sont les coordonnées de l'origine du repère 1 dans le repère 2, alpha est l'angle en degré qu'il y a entre le repère 2 et le repère 1
    new_x = []
    new_y = []
    for i in range(0,len(x)):
        comp_x = x[i] * math.cos(alpha*math.pi/180) - y[i] * math.sin(alpha*math.pi/180) + x_centre
        comp_y = x[i] * math.sin(alpha*math.pi/180) + y[i] * math.cos(alpha*math.pi/180) + y_centre
        new_x = new_x + [comp_x]
        new_y = new_y + [comp_y]
    return new_x, new_y

def table_jeu(): # dessine la table de jeu (en cm)
    x = [0, 200, 200, 0, 0]
    y = [0, 0, 300, 300, 0]
    return plt.plot(x,y)

def objectif(objectif):
    # x_but, y_but sont les coordonnées de l'objectif à atteindre dans le repère de la table
    x_but = objectif[0]
    y_but = objectif[1]
    x = [x_but-2, x_but, x_but, x_but, x_but, x_but+2]
    y = [y_but, y_but, y_but+2, y_but-2, y_but, y_but]
    return plt.plot(x,y)

def obstacle(x_obstacle, y_obstacle, longueur, largeur, alpha):
    # x_obstacle, y_obstacle sont les coordonnées du centre de l'obstacle dans le repère de la table, alpha est l'angle qu'il y a entre le repère de l'obstacle et celui de la table
    x = [-longueur/2, -longueur/2, longueur/2, longueur/2, -longueur/2]
    y = [-largeur/2, largeur/2, largeur/2, -largeur/2, -largeur/2]
    [x_table, y_table] = change_repere(x,y,x_obstacle,y_obstacle,alpha)
    plt.plot(x_table,y_table)    
    return x_table,y_table

def robot(x_robot, y_robot, alpha, points_robot): 
    x =[]
    y = []
    for i in range(0,len(points_robot)):
        x = x + [points_robot[i][0]]
        y = y + [points_robot[i][1]]
        i = i + 1
    x = x + [x[0]]
    y = y + [y[0]]
    [x_table, y_table] = change_repere(x,y,x_robot,y_robot,alpha)
    return plt.plot(x_table,y_table)

def pts_capt_table(pos_capteurs, Robot):
    n = 50 # distance maxiamle pour laquelle le capteur est jugé fiable
    # Création de la liste qui contient tous les points visibles par les différents capteurs
    capt_table = [[[0 for i in range (n)] for j in range(2)] for k in range (nb_capt)]
    for l in range (0, nb_capt):
        y = np.arange(0,n+1,0.5)
        x = zeros(len(y))
        [x_mod, y_mod] = change_repere(x, y, pos_capteurs[l][1], pos_capteurs[l][2], pos_capteurs[l][3])
        [x_mod, y_mod] = change_repere(x_mod, y_mod, Robot[0], Robot[1], Robot[2])
        capt_table[l][0] = x_mod
        capt_table[l][1] = y_mod     
    return capt_table

def points_obstacle(x_coins, y_coins):
    # x_coins, y_coins sont des listes contenant les coordonnées dans le repère de la table des coins d'une forme quelconque, calcul_points renvoie deux listes contenant les abscisses et ordonnées des points entre les coins
    x_points = []
    y_points = []
    n = len(x_coins)
    ## Calcul des pentes entre chaque coin de l'obstacle
    a = []
    b = []
    for i in range (0,n-1):
        if x_coins[i+1]-x_coins[i] != 0:
            a = a + [(y_coins[i+1]-y_coins[i])/(x_coins[i+1]-x_coins[i])]
            b = b + [y_coins[i] - x_coins[i]*a[i]]
        else:
            a = a + [(y_coins[i+1]-y_coins[i])/((x_coins[i+1]+0.001)-x_coins[i])]
            b = b + [y_coins[i] - x_coins[i]*a[i]]
    ## Remplissage des listes x_points et y_points
    for i in range(0, len(x_coins)-1):
        x = x_coins[i]
        y = y_coins[i]
        while min(x_coins[i], x_coins[i+1]) <= x  <= max(x_coins[i], x_coins[i+1]) and  min(y_coins[i],y_coins[i+1]) <= y <= max(y_coins[i],y_coins[i+1]):
            x_points = x_points + [x]
            y_points = y_points + [y]
            if x_coins[i] == x_coins[i+1]:
                x = x_coins[i]
                y = y + np.sign(y_coins[i+1]-y_coins[i])*0.5
            else:
                x = x + np.sign(x_coins[i+1]-x_coins[i])*0.5
                y = a[i]*x + b[i]            
    return x_points, y_points
    
def test_dist(distance):
    if 0 < distance < 49:
        return 1
    else:
        return 0

def detection_capteur(pts_obstacle, pts_capteurs, coord_capteur):
    eps = 1
    dist_obstacle = 50
    for k in range(nb_capt):
        for i in range (0, len(pts_obstacle[0])):
            if min(pts_capteurs[k][0][0], pts_capteurs[k][0][len(pts_capteurs[k][0])-1]) <= pts_obstacle[0][i] <= max(pts_capteurs[k][0][0], pts_capteurs[k][0][len(pts_capteurs[k][0])-1]) and min(pts_capteurs[k][1][0], pts_capteurs[k][1][len(pts_capteurs[k][1])-1]) <= pts_obstacle[1][i] <= max(pts_capteurs[k][1][0], pts_capteurs[k][1][len(pts_capteurs[k][1])-1]):
                for j in range(0, len(pts_capteurs[k][0])):
                    
                    x = pts_capteurs[k][0][j] - pts_obstacle[0][i]
                    y = pts_capteurs[k][1][j] - pts_obstacle[1][i]
                    distance = math.sqrt(x**2 + y**2)
                    if distance  <= eps:
                        x_detecte = pts_capteurs[k][0][j] - coord_capteur[0]
                        y_detecte = pts_capteurs[k][1][j] - coord_capteur[1]
                        dist_obstacle = math.sqrt(x_detecte**2 + y_detecte**2)
                    j = j + 1
            i = i + 1
        k = k + 1
    return dist_obstacle

def liste_dist(Robot, coins_obstacle, pos_capteurs):
    dist = [[] for i in range(nb_capt)] # contient les distances et l'angle de chaque capteur
    for k in range(len(coins_obstacle)): # coins obstacle contient les coins des différents obstacles présents sur la table
        obstacle = points_obstacle(coins_obstacle[0], coins_obstacle[1])    
        for j in range(nb_capt):
            pts_capteurs = pts_capt_table(pos_capteurs, Robot)
            coord_capt_table = change_repere([pos_capteurs[j][1]], [pos_capteurs[j][2]], Robot[0], Robot[1], Robot[2])
            dist[j] = [detection_capteur(obstacle, pts_capteurs, coord_capt_table), pos_capteurs[j][3]]
            j = j + 1
        k = k + 1
    return dist

def ForceAttraction(Robot, Objectif, beta):
    dist = math.sqrt((Objectif[0]-Robot[0])**2 + (Objectif[1]-Robot[1])**2)
    if dist >= 1:
        x_Fa = beta*(Objectif[0] - Robot[0])/dist
        y_Fa = beta*(Objectif[1] - Robot[1])/dist
    else:
        x_Fa = beta*(Objectif[0] - Robot[0])
        y_Fa = beta*(Objectif[1] - Robot[1])
    return x_Fa, y_Fa

def ForceRepUni(dist_obst, alpha):
    rho_0 = 40 # rayon d'influence de l'obstacle en cm
    gamma = 2 # coefficient venant de la formule
    if (dist_obst <= rho_0):
        Frep = (alpha/dist_obst**2)*(((1/dist_obst) - 1/rho_0)**(gamma-1))
    else:
        Frep = 0
    return Frep
    
def ForceRepulsion(list_dist, alpha):
    n = len(alpha)
    x_rep, y_rep = 0, 0
    for i in range(0, n):
        x_rep = x_rep - ForceRepUni(list_dist[i][0], alpha[i][1])*math.cos(math.radians(list_dist[i][1]))
        y_rep = y_rep - ForceRepUni(list_dist[i][0], alpha[i][1])*math.sin(math.radians(list_dist[i][1]))
    theta_rep = math.atan2(y_rep, x_rep)
    theta_rep = angleWrap(theta_rep)
    norme_rep = math.sqrt((x_rep*x_rep) + (y_rep*y_rep))
    return x_rep, y_rep, norme_rep, theta_rep
    
def ForceEvitement(norme_rep, theta_rep, gamma):
    if -90 <= math.degrees(theta_rep) <= 90: # rotation de -90 degrées
        x_ev = float(gamma)*norme_rep*math.sin(theta_rep)
        y_ev = float(gamma)*norme_rep*(-math.cos(theta_rep))
    else: # rotation de 90 degrées
        x_ev = float(gamma)*norme_rep*(-math.sin(theta_rep))
        y_ev = float(gamma)*norme_rep*math.cos(theta_rep)
    return x_ev, y_ev # dans le repère du robot
        
def ForceResultante(ForceAttr, ForceRep, ForceEv):
    x_res = ForceAttr[0] + ForceRep[0] + ForceEv[0]
    y_res = ForceAttr[1] + ForceRep[1] + ForceEv[1]
    theta_mot = math.atan2(y_res, x_res)
#    theta_mot = angleWrap(theta_mot)
    return x_res, y_res, theta_mot

def Deplacement(ForceRes, Robot, Objectif):
    # ForceRes est la sortie de ForceResultante (x, y, theta_mot)
    # Robot coordonnées du robot dans le repère de la table (x, y, theta)
    # Objectif coordonnées que le robot doit atteindre (x, y, theta)
    # vit_robot = 100 # 1m/s 
    t = 1 # période d'échantiollonnage en seconde
    NewRobot = Robot
    NewRobot[0] = Robot[0] + abs(ForceRes[0])*t*math.cos(ForceRes[2])
    NewRobot[1] = Robot[1] + abs(ForceRes[1])*t*math.sin(ForceRes[2])
    NewRobot[2] = ForceRes[2] - math.pi/2
    return NewRobot

def TrajectoireRobot(InitRobot, Objectif, coins_obstacle, pos_capteurs):
    Robot = InitRobot
    epsilon = 0.1
    n = 0
    robot(Robot[0], Robot[1], Robot[2], points_robot)
    while (abs(Robot[0] - Objectif[0]) >= epsilon) and (abs(Robot[1] - Objectif[1]) >= epsilon) and  (0 <= Robot[0] <= 200 and 0 <= Robot[1] <= 300):
        n = n + 1
        ForceAttr = ForceAttraction(Robot, Objectif, beta)
        dist = liste_dist(Robot, coins_obstacle, pos_capteurs)
        ForceRep = ForceRepulsion(dist, alpha)
        ForceEv = ForceEvitement(ForceRep[2], ForceRep[3], gamma)
        ForceRes = ForceResultante(ForceAttr, ForceRep, ForceEv)
        Robot = Deplacement(ForceRes, Robot, Objectif)
        robot(Robot[0], Robot[1], math.degrees(Robot[2]), points_robot)
        print("Start")
        print(ForceRes)
#        print(Robot[2])
    #print (n)
    return 'Done'

def displayAll(InitRobot,Objectif): #  affiche tout les plots calculés
    table_jeu() # Affichage de la table
    objectif(Objectif) # Affichage de l'objectif
    [x_coins_obstacle, y_coins_obstacle] = obstacle(50,75,30,30,0) # Définition de l'obstacle
    TrajectoireRobot(InitRobot, Objectif, [x_coins_obstacle, y_coins_obstacle], pos_capteurs)
    ## Affichage de toutes les figures tracées
    plt.show()
    return 'Done'
