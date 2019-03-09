### Outil de simulation de trajectoire
## VALROBOTIK - 2018

## Importation des modules utilisés
import tkinter as tk
from tkinter import messagebox
import time
import numpy as np
import matplotlib.pyplot as plt
import math

####################
#### Simulation ####
####################

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

## Coordonnées des capteurs dans le repère du robot ([x,y])
capt_D1 = [10, 6.06]
capt_D2 = [12.4, 3.03]
capt_D3 = [12.4, -3.03]
capt_D4 = [4.5, -17]
capt_G1 = [-10, 6.06]
capt_G2 = [-12.4, 3.03]
capt_G3 = [-12.4, -3.03]
capt_G4 = [-4.5, -17]

# Angle entre les repères associés à chaque capteur et le repère du robot
angle_D1 = 0
angle_G1 = 0
angle_D2 = -60
angle_G2 = 60
angle_D3 = 240
angle_G3 = 120
angle_D4 = 240
angle_G4 = 120

# D1 et G1
xD1 = [capt_D1[0], capt_D1[0]]; yD1 = [capt_D1[1] + 3, capt_D1[1] - 3]
xG1 = [capt_G1[0], capt_G1[0]]; yG1 = [capt_G1[1] + 3, capt_G1[1] - 3]
# D2, G2
xD2 = [0, 0]; yD2 = [3, -3]
[xD2, yD2] = change_repere(xD2, yD2, capt_D2[0], capt_D2[1], angle_D2)
xG2 = [0, 0]; yG2 = [3, -3]
[xG2, yG2] = change_repere(xG2, yG2, capt_G2[0], capt_G2[1], angle_G2)
# D3, G3
xD3 = [0, 0]; yD3 = [3, -3]
[xD3, yD3] = change_repere(xD3, yD3, capt_D3[0], capt_D3[1], angle_D3)
xG3 = [0, 0]; yG3 = [3, -3]
[xG3, yG3] = change_repere(xG3, yG3, capt_G3[0], capt_G3[1], angle_G3)
# D4, G4
xD4 = [0, 0]; yD4 = [3, -3]
[xD4, yD4] = change_repere(xD4, yD4, capt_D4[0], capt_D4[1], angle_D4)
xG4 = [0, 0]; yG4 = [3, -3]
[xG4, yG4] = change_repere(xG4, yG4, capt_G4[0], capt_G4[1], angle_G4)

## Définitions des différents coefficients pour le calcul des différentes forces
alpha_D1 = 10
alpha_D2 = 10
alpha_D3 = 10
alpha_D4 = 10
alpha_G1 = 10
alpha_G2 = 10
alpha_G3 = 10
alpha_G4 = 10
beta = 1000
gamma = 10

def table_jeu(): # dessine la table de jeu
    x = [0, 200, 200, 0, 0]
    y = [0, 0, 300, 300, 0]
    return plt.plot(x,y)

def robot(x_robot, y_robot, alpha): # dessine la forme de KUBZ sur la table de jeu
    # (x_robot, y_robot) est la position du robot dans le repère de la table, alpha est l'angle qu'il y a entre le repère du robot (d'origine (x_robot, y_robot)) et le repère de la table
    x = [-14.15, -10.65, 10.65, 14.15, 3.5, -3.5, -14.15, 14.15,0,0,0]
    y = [0, 6.06, 6.06, 0, -18.475, -18.475, 0,0,0,6.06,-18.475]
    [x_table, y_table] = change_repere(x,y,x_robot,y_robot,alpha)
    return plt.plot(x_table,y_table)

def capteurs(x_robot, y_robot, alpha):
    ## Passage dans le repère de la table
    [xD1_table,yD1_table] = change_repere(xD1,yD1,x_robot,y_robot,alpha)
    [xG1_table,yG1_table] = change_repere(xG1,yG1,x_robot,y_robot,alpha)
    [xD2_table,yD2_table] = change_repere(xD2,yD2,x_robot,y_robot,alpha)
    [xG2_table,yG2_table] = change_repere(xG2,yG2,x_robot,y_robot,alpha)
    [xD3_table,yD3_table] = change_repere(xD3,yD3,x_robot,y_robot,alpha)
    [xG3_table,yG3_table] = change_repere(xG3,yG3,x_robot,y_robot,alpha)
    [xD4_table,yD4_table] = change_repere(xD4,yD4,x_robot,y_robot,alpha)
    [xG4_table,yG4_table] = change_repere(xG4,yG4,x_robot,y_robot,alpha)
    ## Affichage
    return plt.plot(xD1_table,yD1_table,'b'), plt.plot(xG1_table,yG1_table,'b'), plt.plot(xD2_table,yD2_table,'b'), plt.plot(xG2_table,yG2_table,'b'), plt.plot(xD3_table,yD3_table,'b'), plt.plot(xG3_table,yG3_table,'b'), plt.plot(xD4_table,yD4_table,'b'), plt.plot(xG4_table,yG4_table,'b')

def objectif(x_but, y_but):
    # x_but, y_but sont les coordonnées de l'objectif à atteindre dans le repère de la table
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

def points_obstacle(x_coins, y_coins):
    # x_coins, y_coins sont des listes contenant les coordonnées dans le repère de la table des coins d'une forme quelconque, calcul_points renvoie deux listes contenant les abscisses et ordonnées des points entre les coins
    x_points = []
    y_points = []
    n = len(x_coins)
    nb_points = 0
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

def points_capteurs(x_robot,y_robot,alpha):
    n = 50 # distance maximale de détection
    ## On écrit d'abord les coordonnées des différents points dans le repère du robot, on les passera dans le repère de la table après
    ## D1 (déjà dans le repère du robot)
    y_points_D1 = capt_D1[1] + np.arange(0,n+1,0.5)
    x_points_D1 = capt_D1[0] * np.array(ones(len(y_points_D1)))
    ## G1 (déjà dans le repère du robot)
    y_points_G1 = capt_G1[1] + np.arange(0,n+1,0.5)
    x_points_G1 = capt_G1[0] * np.array(ones(len(y_points_G1)))
    ## D2
    y_points_D2 = np.arange(0,n+1,0.5)
    x_points_D2 = zeros(len(y_points_D2))
    [x_points_D2, y_points_D2] = change_repere(x_points_D2, y_points_D2, capt_D2[0], capt_D2[1], angle_D2)
    ## G2
    y_points_G2 = np.arange(0,n+1,0.5)
    x_points_G2 = zeros(len(y_points_G2))
    [x_points_G2, y_points_G2] = change_repere(x_points_G2, y_points_G2, capt_G2[0], capt_G2[1], angle_G2)
    ## D3
    y_points_D3 = np.arange(0,n+1,0.5)
    x_points_D3 = zeros(len(y_points_D3))
    [x_points_D3, y_points_D3] = change_repere(x_points_D3, y_points_D3, capt_D3[0], capt_D3[1], angle_D3)
    ## G3
    y_points_G3 = np.arange(0,n+1,0.5)
    x_points_G3 = zeros(len(y_points_G3))
    [x_points_G3, y_points_G3] = change_repere(x_points_G3, y_points_G3, capt_G3[0], capt_G3[1], angle_G3)
    ## D4
    y_points_D4 = np.arange(0,n+1,0.5)
    x_points_D4 = zeros(len(y_points_D4))
    [x_points_D4, y_points_D4] = change_repere(x_points_D4, y_points_D4, capt_D4[0], capt_D4[1], angle_D4)
    ## G4
    y_points_G4 = np.arange(0,n+1,0.5)
    x_points_G4 = zeros(len(y_points_G4))
    [x_points_G4, y_points_G4] = change_repere(x_points_G4, y_points_G4, capt_G4[0], capt_G4[1], angle_G4)
    
    ## On passe ensuite tous ces points dans le repère de la table
    D1 = change_repere(x_points_D1, y_points_D1, x_robot, y_robot, alpha)
    D2 = change_repere(x_points_D2, y_points_D2, x_robot, y_robot, alpha)
    D3 = change_repere(x_points_D3, y_points_D3, x_robot, y_robot, alpha)
    D4 = change_repere(x_points_D4, y_points_D4, x_robot, y_robot, alpha)
    G1 = change_repere(x_points_G1, y_points_G1, x_robot, y_robot, alpha)
    G2 = change_repere(x_points_G2, y_points_G2, x_robot, y_robot, alpha)
    G3 = change_repere(x_points_G3, y_points_G3, x_robot, y_robot, alpha)
    G4 = change_repere(x_points_G4, y_points_G4, x_robot, y_robot, alpha)
    return D1, D2, D3, D4, G1, G2, G3, G4
    
def test_dist(distance):
    if 0 < distance < 49:
        return 1
    else:
        return 0

def detection_capteur(x_points_obstacle, y_points_obstacle, x_points_capteur, y_points_capteur, x_capteur, y_capteur):
    eps = 1
    dist_obstacle = 50
    for i in range (0, len(x_points_obstacle)):
        if min(x_points_capteur[0], x_points_capteur[len(x_points_capteur)-1]) <= x_points_obstacle[i] <= max(x_points_capteur[0], x_points_capteur[len(x_points_capteur)-1]) and min(y_points_capteur[0], y_points_capteur[len(y_points_capteur)-1]) <= y_points_obstacle[i] <= max(y_points_capteur[0], y_points_capteur[len(y_points_capteur)-1]):
            for j in range(0, len(x_points_capteur)):
                x = x_points_capteur[j] - x_points_obstacle[i]
                y = y_points_capteur[j] - y_points_obstacle[i]
                distance = math.sqrt(x**2 + y**2)
                if distance  <= eps:
                    x_detecte = x_points_capteur[j] - x_capteur
                    y_detecte = y_points_capteur[j] - y_capteur
                    dist_obstacle = math.sqrt(x_detecte**2 + y_detecte**2)
                else:
                    j = j + 1
        i = i + 1
    return dist_obstacle

def detection_obstacle(x_robot, y_robot, alpha, x_coins_obstacle, y_coins_obstacle):
    ## Coordonnées des capteurs dans le repère de la table
    [x_capt_D1, y_capt_D1] = change_repere([capt_D1[0]], [capt_D1[1]], x_robot, y_robot, alpha)
    [x_capt_D2, y_capt_D2] = change_repere([capt_D2[0]], [capt_D2[1]], x_robot, y_robot, alpha)
    [x_capt_D3, y_capt_D3] = change_repere([capt_D3[0]], [capt_D3[1]], x_robot, y_robot, alpha)
    [x_capt_D4, y_capt_D4] = change_repere([capt_D4[0]], [capt_D4[1]], x_robot, y_robot, alpha)
    [x_capt_G1, y_capt_G1] = change_repere([capt_G1[0]], [capt_G1[1]], x_robot, y_robot, alpha)
    [x_capt_G2, y_capt_G2] = change_repere([capt_G2[0]], [capt_G2[1]], x_robot, y_robot, alpha)
    [x_capt_G3, y_capt_G3] = change_repere([capt_G3[0]], [capt_G3[1]], x_robot, y_robot, alpha)
    [x_capt_G4, y_capt_G4] = change_repere([capt_G4[0]], [capt_G4[1]], x_robot, y_robot, alpha)

    [x_points_obstacle, y_points_obstacle] = points_obstacle(x_coins_obstacle, y_coins_obstacle)
    [D1, D2, D3, D4, G1, G2, G3, G4] = points_capteurs(x_robot, y_robot, alpha)
    
    ## Calcul des  distances renvoyées par les sharps
    d_D1 = detection_capteur(x_points_obstacle, y_points_obstacle, D1[0], D1[1], x_capt_D1, y_capt_D1)
    d_D2 = detection_capteur(x_points_obstacle, y_points_obstacle, D2[0], D2[1], x_capt_D2, y_capt_D2)
    d_D3 = detection_capteur(x_points_obstacle, y_points_obstacle, D3[0], D3[1], x_capt_D3, y_capt_D3)
    d_D4 = detection_capteur(x_points_obstacle, y_points_obstacle, D4[0], D4[1], x_capt_D4, y_capt_D4)
    d_G1 = detection_capteur(x_points_obstacle, y_points_obstacle, G1[0], G1[1], x_capt_G1, y_capt_G1)
    d_G2 = detection_capteur(x_points_obstacle, y_points_obstacle, G2[0], G2[1], x_capt_G2, y_capt_G2)
    d_G3 = detection_capteur(x_points_obstacle, y_points_obstacle, G3[0], G3[1], x_capt_G3, y_capt_G3)
    d_G4 = detection_capteur(x_points_obstacle, y_points_obstacle, G4[0], G4[1], x_capt_G4, y_capt_G4)
    return d_D1, d_D2, d_D3, d_D4, d_G1, d_G2, d_G3, d_G4
    
def forceRepulsion(d_D1, d_D2, d_D3, d_D4, d_G1, d_G2, d_G3, d_G4): #d_Di et d_Gi sont les distances obtenus via les tensions renvoyées par les différents capteurs sharps montés sur le robot
    # composantes x et y dans le repère du robot
    composante_x = - (math.cos(math.pi/6) * ((alpha_D2*test_dist(d_D2)/d_D2**2 + alpha_D3*test_dist(d_D3)/d_D3**2 + alpha_D4*test_dist(d_D4)/d_D4**2) - (alpha_G2*test_dist(d_G2)/d_G2**2 + alpha_G3*test_dist(d_G3)/d_G3**2 + alpha_G4*test_dist(d_G4)/d_G4**2)))
    composante_y = - (alpha_D1*test_dist(d_D1)/d_D1**2 + alpha_G1*test_dist(d_G1)/d_G1**2 + math.sin(math.pi/6) * (alpha_D2*test_dist(d_D2)/d_D2**2 - alpha_D3*test_dist(d_D3)/d_D3**2 - alpha_D4*test_dist(d_D4)/d_D4**2 + alpha_G2*test_dist(d_G2)/d_G2**2 - alpha_G3*test_dist(d_G3)/d_G3**2 - alpha_G4*test_dist(d_G4)/d_G4**2))
    return composante_x, composante_y

def forceAttraction(x_robot, y_robot, x_but, y_but):
    # calcule les composantes de la force d'attraction, celles-ci sont dans le repère de la table
    x_vect = x_but - x_robot
    y_vect = y_but - y_robot
    distance = math.sqrt(x_vect**2 + y_vect**2)
    angle = math.atan(y_vect/x_vect)*180/math.pi
    composante_x = beta*math.cos(angle*math.pi/180)/(distance**2) + x_robot
    composante_y = beta*math.sin(angle*math.pi/180)/(distance**2) + y_robot
    return composante_x, composante_y

def forceEvitement(x_repulsion, y_repulsion, x_repulsion_table, y_repulsion_table, x_attraction, y_attraction):
    if x_repulsion_table*x_attraction + y_repulsion_table*y_attraction == 0:
        angle = 90 # angle est l'angle entre la force de répulsion et la force d'attraction
    else:
        var = math.sqrt(x_repulsion_table**2 + y_repulsion_table**2)*math.sqrt(x_attraction**2+y_attraction**2)/(x_repulsion_table*x_attraction+y_repulsion_table*y_attraction)
        print('valeur', var)
##    if  angle <= 180 :
##        angle_evitement = -90
##    else:
##        angle_evitement = 90
    [x_evitement, y_evitement] = change_repere([x_repulsion], [y_repulsion], 0, 0, 90)
    return x_evitement, y_evitement

def calcul_forces(x_robot, y_robot, alpha, x_coins_obstacle, y_coins_obstacle, x_but, y_but):
    [d_D1, d_D2, d_D3, d_D4, d_G1, d_G2, d_G3, d_G4] = detection_obstacle(x_robot, y_robot, alpha, x_coins_obstacle, y_coins_obstacle)
    # Répulsion
    [x_repulsion, y_repulsion] = forceRepulsion(d_D1, d_D2, d_D3, d_D4, d_G1, d_G2, d_G3, d_G4)
    [x_repulsion_table, y_repulsion_table] = change_repere([x_repulsion], [y_repulsion], x_robot, y_robot, alpha)
    x_repulsion_table = list2int(x_repulsion_table)
    y_repulsion_table = list2int(y_repulsion_table)
    
    # Attraction
    [x_attraction, y_attraction] = forceAttraction(x_robot, y_robot, x_but, y_but)
    
    # Evitement
    [x_evitement, y_evitement] = forceEvitement(x_repulsion, y_repulsion, x_repulsion_table, y_repulsion_table,x_attraction, y_attraction)
    [x_evitement_table, y_evitement_table] = change_repere(x_evitement, y_evitement, x_robot, y_robot, alpha)
    x_evitement_table = list2int(x_evitement_table)
    y_evitement_table = list2int(y_evitement_table)
   
    # Résultante des forces
    x_resultante = (x_repulsion_table-x_robot) + (x_attraction-x_robot) + (x_evitement_table-x_robot)
    y_resultante = (y_repulsion_table-y_robot) + (y_attraction-y_robot) + (y_evitement_table-y_robot)

    print('x', 'répulsion', x_repulsion_table-x_robot, 'attraction', x_attraction-x_robot, 'évitement', x_evitement_table-x_robot, 'résultante', x_resultante)
    print('y', 'répulsion', y_repulsion_table-y_robot, 'attraction', y_attraction-y_robot, 'évitement', y_evitement_table-y_robot, 'résultante', y_resultante)
    # Tracé des figures
##    plt.plot([x_robot,x_repulsion_table], [y_robot, y_repulsion_table], [x_robot, x_attraction], [y_robot,y_attraction], [x_robot, x_evitement_table], [y_robot, y_evitement_table], [x_robot, x_resultante+x_robot], [y_robot, y_resultante+y_robot])
    return x_resultante, y_resultante

def deplacement_robot(x_resultante, y_resultante, x_robot, y_robot, x_but, y_but):
    angle = math.atan(y_resultante/x_resultante)
    if math.sqrt((x_but-x_robot)**2 + (y_but-y_robot)**2) >= 10:
        dist_deplacement = 10
    else:
        dist_deplacement = math.sqrt((x_but-x_robot)**2 + (y_but-y_robot)**2)
    print('longueur déplacement', dist_deplacement)
    new_x_robot = x_robot + dist_deplacement*math.cos(angle)
    new_y_robot = y_robot + dist_deplacement*math.sin(angle)
    angle = angle * 180/math.pi - 90
    return new_x_robot, new_y_robot, angle

def trajectoire_robot(x_init_robot, y_init_robot, angle_init,x_but, y_but, x_coins_obstacle, y_coins_obstacle):
    x_robot = x_init_robot
    y_robot = y_init_robot
    angle = angle_init
    epsilon = 0.1
    n = 1
    while abs(x_robot - x_but) >= epsilon and abs(y_robot-y_but) >= epsilon and  0 <= x_robot <= 200 and 0 <= y_robot <= 300:
        print('position numero', n)
        [x_resultante, y_resultante] = calcul_forces(x_robot, y_robot, angle, x_coins_obstacle, y_coins_obstacle, x_but, y_but)
        [x_robot, y_robot, angle] = deplacement_robot(x_resultante, y_resultante, x_robot, y_robot, x_but, y_but)
        print('x_robot', x_robot, 'y_robot', y_robot, 'angle', angle)
        robot(x_robot, y_robot, angle)
        n = n + 1
    return 'Done'

def displayAll(x_robot, y_robot, x_but, y_but, alpha): #  affiche tout les plots calculés
    table_jeu() # Affichage de la table
    objectif(x_but,y_but) # Affichage de l'objectif
    robot(x_robot, y_robot, alpha) # Affichage du robot en position initiale
    capteurs(x_robot,y_robot,alpha) # Affichage des capteurs sur le robot
    [x_coins_obstacle, y_coins_obstacle] = obstacle(50,75,30,30,0) # Définition de l'obstacle
    trajectoire_robot(x_robot, y_robot, alpha, x_but, y_but, x_coins_obstacle, y_coins_obstacle);
    ## Affichage de toutes les figures tracées
    plt.show()
    return 'Done'


#################
### Affichage ###
#################

coord_x = []
coord_y = []
i = 0

def start_simulation():
    if len(coord_x) == 0 or len(coord_y) == 0:
        messagebox.showwarning(title = "Warning", message = "Vous devez renseigner des points")
    else:
        # Position initiale du robot
        x_robot_init = entree_x.get()
        y_robot_init = entree_y.get()
        angle_robot_init = entree_angle.get()
        x_objectif = coord_x[0]
        y_objectif = coord_y[0]
        displayAll(x_robot_init, y_robot_init, x_objectif, y_objectif, angle_robot_init)
    return 'Done'

def mod_liste(L, i): # enlève le i ème élément de la liste L
    L = L[:i-1] + L[i:]
    return L

def clic_souris(event):
    global coord_x, coord_y # coordonnées des étapes imposées au robot
    global i
    global bouton_step
    coord_clic = [event.x, event.y]
    coord_x = coord_x + [coord_clic[0]]
    coord_y = coord_y + [coord_clic[1]]
    print("données clic-souris", coord_clic)
    print("coord_x", coord_x, "coord_y", coord_y)
    print(i)
    step_window = tk.Frame(subwindow_3, width = 300, borderwidth = 2, relief = tk.GROOVE)
    step_window.pack(side = tk.TOP)
    tk.Label(step_window, text = '%s | %s' %(coord_clic[0], coord_clic[1]), anchor = tk.W, borderwidth = 2).pack()
    bouton_step = tk.Button(step_window, anchor = tk.CENTER, text = "Suppr", fg = 'navy').pack()
    
def open_robots():
    robots_window = tk.Toplevel(window)
    robots_window.title('Robots')
    print('ouvre robots')

window = tk.Tk() # création de la fenêtre principale
window.title('Outil de simulation de la trajectoire') # titre de la fenêtre principale

## création du menu déroulant
menubar = tk.Menu(window)

## création des sous-menus
menu_fichier = tk.Menu(menubar, tearoff = 0)
menu_fichier.add_command(label = "Importer trajectoire") # permet d'importer un fichier contenant les points d'une trajectoire déjà testée
menu_fichier.add_command(label = "Exporter trajectoire") # permet d'exporter un fichier contenant la trajectoire testée
menu_fichier.add_command(label = "Robots", command = open_robots()) # Ouvre une fenêtre affichant le/les robots pouvant être utilisés, on peut en éditer de nouveau en renseignant les dimensions, la position des capteurs, etc
menu_fichier.add_separator()
menu_fichier.add_command(label = "Exit", command = menu_fichier.quit)
menubar.add_cascade(label = "Fichier", menu = menu_fichier)

menu_params_avances = menubar.add_command(label = "Paramètres avancés") # Ouvre une fenêtre où l'on pourra modifier les paramètres (poids) utilisés dans le calcul des différentes forces -> il faut aussi prévoir une configuration par défaut
menu_help = menubar.add_command(label = "Aide") # Ouvrira le PDF décrivant le fonctionnement du programme et comment l'utiliser

## création d'une première sous-fenêtre, elle contiendra la carte intéractive de la table
subwindow_1 = tk.Frame(window, borderwidth = 2, relief = tk.GROOVE)
subwindow_1.pack(side = tk.LEFT)

## création d'une deuxième sous fenêtre, elle contiendra les paramètres de la simulation (durée, position initiale du robot, éventuellement le choix entre plusieurs robots)
subwindow_2 = tk.Frame(window, borderwidth = 2, relief = tk.GROOVE)
subwindow_2.pack(side = tk.RIGHT)

## création d'une troisième sous-fenêtre qui contiendra les différents étapes de la trajectoire du robot, le tableau se remplira au fur et à mesure du choix des étapes
subwindow_3 = tk.Frame(window, width = 500, borderwidth = 2, relief = tk.GROOVE)
subwindow_3.pack()

## création des widgets Labels et des différents bouttons et entrées
# fenêtre principale
tk.Button(window, text = 'Démarrer simulation', fg = 'navy', command = start_simulation).pack(side = tk.BOTTOM, padx = 10, pady = 10)

# première sous-fenêtre
# ouverture de la photo de la table
table = tk.PhotoImage(file="table_cr19.PNG")
largeur = table.width()
hauteur = table.height()
canvas = tk.Canvas(subwindow_1, width = largeur, height = hauteur)
canvas.create_image(largeur/2,hauteur/2,image = table)
canvas.focus_set()
canvas.bind("<Button-1>", clic_souris)
canvas.pack()

# deuxième sous-fenêtre
# Durée
label_duree = tk.Label(subwindow_2, text = 'Durée de la simulation', fg = 'navy')
label_duree.pack()
value_duree = tk.StringVar()
value_duree.set("Durée")
entree_duree = tk.Entry(subwindow_2, textvariable = value_duree, width = 20)
entree_duree.pack()

# Position initiale du robot (x, y, angle x->y)
label_position = tk.Label(subwindow_2, text = 'Position initiale du robot', fg = 'navy')
label_position.pack()

value_x = tk.StringVar()
value_x.set("x")
entree_x = tk.Entry(subwindow_2, textvariable = value_x, width = 20)
entree_x.pack()

value_y = tk.StringVar()
value_y.set("y")
entree_y = tk.Entry(subwindow_2, textvariable = value_y, width = 20)
entree_y.pack()

value_angle = tk.StringVar()
value_angle.set("angle")
entree_angle = tk.Entry(subwindow_2, textvariable = value_angle, width = 20)
entree_angle.pack()

# Choix du robot
value_robot = tk.StringVar()
button_petit = tk.Radiobutton(subwindow_2, text = "Petit robot", variable = value_robot, value = 1)
button_gros = tk.Radiobutton(subwindow_2, text = "Gros robot", variable = value_robot, value = 2)
button_petit.pack()
button_gros.pack()

# troisième sous-fenêtre
label_etapes = tk.Label(subwindow_3, text = "Etapes de la trajectoire", fg = 'navy')
label_etapes.pack()

window.config(menu = menubar)
window.mainloop()

