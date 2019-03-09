### Outil de simulation de trajectoire
## VALROBOTIK - 2018

"""
@author: Loic Dietz

NOTE : Ce programme dispose d'une interface graphique
"""

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
        print(x)
        y = y + [points_robot[i][1]]
        print(y)
        i = i + 1
    x = x + [x[0]]
    y = y + [y[0]]
    [x_table, y_table] = change_repere(x,y,x_robot,y_robot,alpha)
    print(x_table)
    print(y_table)
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
            if min(pts_capteurs[k][0][0], pts_capteurs[k][0][len(pts_capteurs[k][0])]) <= pts_obstacle[0][i] <= max(pts_capteurs[k][0][0], pts_capteurs[k][0][len(pts_capteurs[k][0])-1]) and min(pts_capteurs[k][1][0], pts_capteurs[k][1][len(pts_capteurs[k][1])-1]) <= pts_obstacle[1][i] <= max(pts_capteurs[k][1][0], pts_capteurs[k][1][len(pts_capteurs[k][1])-1]):
                for j in range(0, len(pts_capteurs[k][0])):
                    x = pts_capteurs[k][0][j] - pts_obstacle[0][i]
                    y = pts_capteurs[k][1][j] - pts_obstacle[1][i]
                    distance = math.sqrt(x**2 + y**2)
                    if distance  <= eps:
                        x_detecte = pts_capteurs[0][j] - coord_capteur[0]
                        y_detecte = pts_capteurs[1][j] - coord_capteur[1]
                        dist_obstacle = math.sqrt(x_detecte**2 + y_detecte**2)
                    else:
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
            dist[j] = [detection_capteur(obstacle, pts_capteurs, coord_capt_table), pos_capteurs[j][2]]
            j = j + 1
        k = k + 1
    return dist

def ForceAttraction(Robot, Objectif, beta):
    dist = math.sqrt((Objectif(0)-Robot(0))^2 + (Objectif(1)-Robot(1))^2)
    if dist >= 1:
        x_Fa = beta*(Objectif(0) - Robot(0))/dist
        y_Fa = beta*(Objectif(1) - Robot(1))/dist
    else:
        x_Fa = beta*(Objectif(0) - Robot(0))
        y_Fa = beta*(Objectif(1) - Robot(1))
    return x_Fa, y_Fa

def ForceRepUni(dist_obst, alpha):
    rho_0 = 0.2 # rayon d'influence de l'obstacle en m
    gamma = 2 # coefficient venant de la formule
    if (dist_obst <= rho_0):
        Frep = (alpha/dist_obst^2)*(((1/dist_obst) - 1/rho_0)^(gamma-1))
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
    norme_rep = math.sqrt((x_rep^2) + (y_rep^2))
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
    theta_mot = math.atan2(y_res, x_res) + math.pi/2
    return x_res, y_res, theta_mot

def Deplacement(ForceRes, Robot, Objectif):
    # ForceRes est la sortie de ForceResultante (x, y, theta_mot)
    # Robot coordonnées du robot dans le repère de la table (x, y, theta)
    # Objectif coordonnées que le robot doit atteindre (x, y, theta)
    # vit_robot = 100 # 1m/s 
    t = 0.02 # période d'échantiollonnage en seconde
    NewRobot = Robot
    NewRobot[0] = Robot[0] + ForceRes[0]*t*math.cos(ForceRes[2])
    NewRobot[1] = Robot[1] + ForceRes[1]*t*math.sin(ForceRes[2])
    NewRobot[2] = ForceRes[2]
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
        robot(Robot[0], Robot[1], Robot[2], points_robot)
    print (n)
    return 'Done'

def displayAll(InitRobot,Objectif): #  affiche tout les plots calculés
    table_jeu() # Affichage de la table
    objectif(Objectif) # Affichage de l'objectif
    [x_coins_obstacle, y_coins_obstacle] = obstacle(50,75,30,30,0) # Définition de l'obstacle
    TrajectoireRobot(InitRobot, Objectif, [x_coins_obstacle, y_coins_obstacle], pos_capteurs)
    ## Affichage de toutes les figures tracées
    plt.show()
    return 'Done'


#################
### Affichage ###
#################

coord_x = []
coord_y = []
i = 0

## Définition des fonctions utilisées dans l'affichage
def start_simulation():
    if len(coord_x) == 0 or len(coord_y) == 0:
        messagebox.showwarning(title = "Warning", message = "Vous devez renseigner des points")
    else:
        # Position initiale du robot
        InitRobot = [entree_x.get(), entree_y.get(), entree_angle.get()]
        Objectif = [int(coord_x[0]), int(coord_y[0])]
        displayAll(InitRobot, Objectif)
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
    step_window = tk.Frame(subwindow_3, width = 300, borderwidth = 2, relief = tk.GROOVE)
    step_window.pack(side = tk.TOP)
    def delete_step():
        step_window.destroy()
        
    tk.Label(step_window, text = '%s | %s' %(coord_clic[0], coord_clic[1]), anchor = tk.W, borderwidth = 2).pack()
    bouton_step = tk.Button(step_window, anchor = tk.CENTER, text = "Suppr", fg = 'navy', command = delete_step).pack()

def infos_capteurs():
    global table_capt
    global nb_capt
    global table_coeff
    # Récupération du nombre de capteurs
    nb_capt = int(nb_capt_robot.get())
    # Création des entrées pour récupérer les coordonnées des capteurs dans le repère du robot
    tableau_valeurs = [[0 for j in range(6)] for k in range (nb_capt)] 
    tableau_coeff = [[0 for j in range(4)] for k in range (nb_capt)] 
    for i in range (0,nb_capt):
        tableau_valeurs[i][0] = tk.DoubleVar()
        tableau_valeurs[i][0].set("x")
        tableau_valeurs[i][1] = tk.Entry(capt_window, textvariable = tableau_valeurs[i][0], width = 10)
        tableau_valeurs[i][1].grid(row = i+2, column = 0)
        
        tableau_valeurs[i][2] = tk.DoubleVar()
        tableau_valeurs[i][2].set("y")
        tableau_valeurs[i][3] = tk.Entry(capt_window, textvariable = tableau_valeurs[i][2], width = 10)
        tableau_valeurs[i][3].grid(row = i+2, column = 1)
        
        tableau_valeurs[i][4] = tk.DoubleVar()
        tableau_valeurs[i][4].set("angle")
        tableau_valeurs[i][5] = tk.Entry(capt_window, textvariable = tableau_valeurs[i][4], width = 10)
        tableau_valeurs[i][5].grid(row = i+2, column = 2)
        
        nom = 'capt'+ str(i)
        tableau_coeff[i][0] = tk.StringVar()
        tableau_coeff[i][0].set(nom)
        tableau_coeff[i][1] = tk.Entry(coeff_alpha_window, textvariable = tableau_coeff[i][0], width = 10)
        tableau_coeff[i][1].grid(row = i+1, column = 0)
        tableau_coeff[i][2] = tk.DoubleVar()
        tableau_coeff[i][2].set(1.0)
        tableau_coeff[i][3] = tk.Entry(coeff_alpha_window, textvariable = tableau_coeff[i][2], width = 10)
        tableau_coeff[i][3].grid(row = i+1, column = 1)
        
    table_capt = tableau_valeurs
    table_coeff = tableau_coeff

def infos_points():
    global table_points
    global nb_points
    # Récupération du nombre de points du périmètre du robot
    nb_points = nb_pts_robot.get()
    # Création des entrées pour récupérer les coordonnées des extrémités du robot dans le repère du robot
    coord_pts = [[0 for j in range(4)] for k in range (nb_points)] # [x, y]
    for i in range (0,nb_points):
        coord_pts[i][0] = tk.DoubleVar()
        coord_pts[i][0].set("x")
        coord_pts[i][1] = tk.Entry(pts_robot_window, textvariable = coord_pts[i][0], width = 10)
        coord_pts[i][1].grid(row = i+2, column = 0)
        coord_pts[i][2] = tk.DoubleVar()
        coord_pts[i][2].set("y")
        coord_pts[i][3] = tk.Entry(pts_robot_window, textvariable = coord_pts[i][2], width = 10)
        coord_pts[i][3].grid(row = i+2, column = 2)
    table_points = coord_pts

def recup_infos():
    global pts_robot
    global name
    global pos_capteurs
    global points_robot
    global alpha
    global beta
    global gamma
    global nb_points
    global nb_capt
    # Récupération des infos sur les capteurs et les points du périmètre du robot
    nb_points = nb_pts_robot.get()
    nb_capt = nb_capt_robot.get()
    pos_capteurs = [[0 for j in range(4)] for k in range (nb_capt)]
    alpha = [[0 for j in range(2)] for k in range (nb_capt)] # contient les poids associés aux capteurs
    for i in range (0,nb_capt):
        nom = 'capt'+ str(i)
        pos_capteurs[i][0] = nom
        pos_capteurs[i][1] = table_capt[i][1].get()
        pos_capteurs[i][2] = table_capt[i][3].get()
        pos_capteurs[i][3] = table_capt[i][5].get()
        #Transformation en float
        pos_capteurs[i][1] = float(pos_capteurs[i][1])
        pos_capteurs[i][2] = float(pos_capteurs[i][2])
        pos_capteurs[i][3] = float(pos_capteurs[i][3])
        # Récupération des poids associés à chaque capteur
        alpha[i][0] = nom
        alpha[i][1] = float(table_coeff[i][3].get())
        
    points_robot = [[0 for j in range(2)] for k in range (nb_points)]
    for l in range (0, nb_points):
        points_robot[l][0] = table_points[l][1].get()
        points_robot[l][1] = table_points[l][3].get()
        # Transformation en entier
        points_robot[l][0] = float(points_robot[l][0])
        points_robot[l][1] = float(points_robot[l][1])
    name = value_name.get()
    # Récupération des valeurs des coefficients beta et gamma
    beta = float(value_beta.get())
    gamma = float(value_gamma.get())   
    
def dessin():
    # On récupère les caractéristiques des capteurs et des points du périimètres du robot
    recup_infos()
    points_dessin = [[0 for j in range(2)] for k in range (nb_points)]
#    print(alpha)
#    print(pos_capteurs)
    #  On retravaille les coordonnées pour l'affichage
    max_x = []
    max_y = []
    for j in range(0, nb_points):
        max_x = max_x + [abs(points_robot[j][0])]
        max_y = max_y + [abs(points_robot[j][1])]
    facteur_x = 100/max(max_x)
    facteur_y = 100/max(max_y)
    for j in range(0, nb_points):
        points_dessin[j][0] = facteur_x*points_robot[j][0] + 125
        points_dessin[j][1] = facteur_y*(-points_robot[j][1]) + 125
    # On retravaille les coordonnées des capteurs pour l'affichage dans le repère du robot avec change_repere
    # Création du Canvas
    fig_robot = tk.Canvas(dessin_window, height = 250, width = 250, bg = 'white')
    fig_robot.pack(side = tk.BOTTOM)
    for i in range(0, nb_capt):
        # [xD2, yD2] = change_repere(xD2, yD2, capt_D2[0], capt_D2[1], angle_D2)
        capt_dessin = change_repere([0, 0], [2, -2], pos_capteurs[i][1], pos_capteurs[i][2], pos_capteurs[i][3])
        fig_robot.create_line((125 + facteur_x*capt_dessin[0][0], 125 + facteur_y*(-capt_dessin[1][0])),(125 + facteur_x*capt_dessin[0][1], 125 + facteur_y*(-capt_dessin[1][1])), fill = 'red')
    # On dessine le robot dans le Canvas fig_robot, taille du canvas 250x250
    for i in range(0, nb_points-1):
        fig_robot.create_line(points_dessin[i], points_dessin[i+1])
    fig_robot.create_line((0,125),(250,125))
    fig_robot.create_line((125,0), (125,250))
    fig_robot.create_line(points_dessin[nb_points-1], points_dessin[0])
    
def save(): # Permet d'enregistrer les coordonnées des capteurs et du périmètre du robot dans un fichier texte
    # Création du fichier texte
    recup_infos()
    nb_points = nb_pts_robot.get()
    nb_capt = nb_capt_robot.get()
    name = value_name.get() + '.txt'
    fichier = open(name, "w")
    # 1ère ligne : Nom de la configuration/du robot
    fichier.write(name + '\n')
    fichier.write(str(nb_points) + '\n')
    fichier.write(str(nb_capt) + '\n')
    fichier.write('Points')
    for i in range(0, nb_points):
        fichier.write('\n' + str(points_robot[i][0]) + ','+ str(points_robot[i][1]))
    fichier.write('\nCapteurs')
    for i in range(0, nb_capt):
        fichier.write('\n' + str(pos_capteurs[i][0]) + ','+ str(pos_capteurs[i][1]) + ','+ str(pos_capteurs[i][2]) + ','+ str(pos_capteurs[i][3]))
    fichier.close

def load():
    name = value_name.get() + '.txt' # On récupère le nom de la configuration que l'on souhaite charger
    fichier = open(name, 'r')
    fichier.readline()
    # On récupère le nombre de points et de capteurs
    nb_points = fichier.readline()
    nb_points = int(nb_points[:len(nb_points)-1])
    nb_capt = fichier.readline()
    nb_capt = int(nb_capt[:len(nb_capt)-1])
    fichier.readline() # On passe la ligne avec marquée Points
    # Maintenant on récupère les informations importantes (capteurs et points)
    # Partie points
    points_robot = []
    for i in range (0, nb_points):
        line = fichier.readline()
        line = line[:len(line)-1]
        points_robot = points_robot + [list(map(float, line.split(',')))] #[x, y]
    # Partie capteurs
    fichier.readline() # On passe la ligne avec marquée Capteurs
    pos_capteurs = []
    for i in range (0, nb_capt):
        nom = 'capt' + str(i)
        line = fichier.readline()
        line = line[len(nom + ','):len(line)-1]
        pos_capteurs = pos_capteurs + [list(map(float, line.split(',')))] #[x, y, angle]
    fichier.close()
    
    # Ecriture dans les tableaux
    nb_pts_robot.set(nb_points)
    nb_capt_robot.set(nb_capt)
    global table_capt
    global table_coeff
    tableau_valeurs = [[0 for j in range(6)] for k in range (nb_capt)]
    tableau_coeff = [[0 for j in range(4)] for k in range (nb_capt)] 
    for i in range (0,nb_capt):
        tableau_valeurs[i][0] = tk.DoubleVar()
        tableau_valeurs[i][0].set(pos_capteurs[i][0])
        tableau_valeurs[i][1] = tk.Entry(capt_window, textvariable = tableau_valeurs[i][0], width = 10)
        tableau_valeurs[i][1].grid(row = i+2, column = 0)
        tableau_valeurs[i][2] = tk.DoubleVar()
        tableau_valeurs[i][2].set(pos_capteurs[i][1])
        tableau_valeurs[i][3] = tk.Entry(capt_window, textvariable = tableau_valeurs[i][2], width = 10)
        tableau_valeurs[i][3].grid(row = i+2, column = 1)
        tableau_valeurs[i][4] = tk.DoubleVar()
        tableau_valeurs[i][4].set(pos_capteurs[i][2])
        tableau_valeurs[i][5] = tk.Entry(capt_window, textvariable = tableau_valeurs[i][4], width = 10)
        tableau_valeurs[i][5].grid(row = i+2, column = 2)    
    
        nom = 'capt'+ str(i)
        tableau_coeff[i][0] = tk.StringVar()
        tableau_coeff[i][0].set(nom)
        tableau_coeff[i][1] = tk.Entry(coeff_alpha_window, textvariable = tableau_coeff[i][0], width = 10)
        tableau_coeff[i][1].grid(row = i+1, column = 0)
        tableau_coeff[i][2] = tk.DoubleVar()
        tableau_coeff[i][2].set(1.0)
        tableau_coeff[i][3] = tk.Entry(coeff_alpha_window, textvariable = tableau_coeff[i][2], width = 10)
        tableau_coeff[i][3].grid(row = i+1, column = 1)
    
    table_capt = tableau_valeurs
    table_coeff = tableau_coeff
    global table_points
    coord_pts = [[0 for j in range(4)] for k in range (nb_points)]
    for i in range (0,nb_points):
        coord_pts[i][0] = tk.DoubleVar()
        coord_pts[i][0].set(points_robot[i][0])
        coord_pts[i][1] = tk.Entry(pts_robot_window, textvariable = coord_pts[i][0], width = 10)
        coord_pts[i][1].grid(row = i+2, column = 0)
        coord_pts[i][2] = tk.DoubleVar()
        coord_pts[i][2].set(points_robot[i][1])
        coord_pts[i][3] = tk.Entry(pts_robot_window, textvariable = coord_pts[i][2], width = 10)
        coord_pts[i][3].grid(row = i+2, column = 2)
    table_points = coord_pts

def clear():
    n = 0
    for item in pts_robot_window.winfo_children():
        n = n + 1
        if n > 3 : 
            item.destroy()
    n = 0
    for item in capt_window.winfo_children():
        n = n + 1
        if n > 3 :
            item.destroy()
    n = 0
    for item in dessin_window.winfo_children():
        n = n + 1
        if n > 5 :
            item.destroy()
    nb_pts_robot.set('')
    nb_capt_robot.set('')
    name_robot.set('')
    
window = tk.Tk() # création de la fenêtre principale
window.title('Outil de simulation de la trajectoire') # titre de la fenêtre principale

## création du menu déroulant
menubar = tk.Menu(window)

## création des sous-menus
menu_fichier = tk.Menu(menubar, tearoff = 0)
menu_fichier.add_command(label = "Importer trajectoire") # permet d'importer un fichier contenant les points d'une trajectoire déjà testée
menu_fichier.add_command(label = "Exporter trajectoire") # permet d'exporter un fichier contenant la trajectoire testée
menu_fichier.add_command(label = "Robots") # Ouvre une fenêtre affichant le/les robots pouvant être utilisés, on peut en éditer de nouveau en renseignant les dimensions, la position des capteurs, etc
menu_fichier.add_separator()
menu_fichier.add_command(label = "Exit", command = menu_fichier.quit)
menubar.add_cascade(label = "Fichier", menu = menu_fichier)

menu_params_avances = menubar.add_command(label = "Paramètres avancés") # Ouvre une fenêtre où l'on pourra modifier les paramètres (poids) utilisés dans le calcul des différentes forces -> il faut aussi prévoir une configuration par défaut
menu_help = menubar.add_command(label = "Aide") # Ouvrira le PDF décrivant le fonctionnement du programme et comment l'utiliser


### Fenêtre contenant la carte, les coordonnées de la position initiale du robot
map_window = tk.Frame(window, borderwidth = 10, relief = tk.RAISED)
map_window.pack(side = tk.LEFT)
## création d'une première sous-fenêtre, elle contiendra la carte intéractive de la table
subwindow_1 = tk.Frame(map_window, borderwidth = 2, relief = tk.GROOVE)
subwindow_1.pack(side = tk.LEFT)

## création d'une deuxième sous fenêtre, elle contiendra les paramètres de la simulation (durée, position initiale du robot, éventuellement le choix entre plusieurs robots)
subwindow_2 = tk.Frame(map_window, borderwidth = 2, relief = tk.GROOVE)
subwindow_2.pack(side = tk.RIGHT)

## création d'une troisième sous-fenêtre qui contiendra les différents étapes de la trajectoire du robot, le tableau se remplira au fur et à mesure du choix des étapes
subwindow_3 = tk.Frame(map_window, width = 500, borderwidth = 2, relief = tk.GROOVE)
subwindow_3.pack()

## création des widgets Labels et des différents bouttons et entrées
# fenêtre principale
tk.Button(map_window, text = 'Démarrer simulation', fg = 'navy', command = start_simulation).pack(side = tk.BOTTOM, padx = 10, pady = 10)

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
value_duree = tk.IntVar()
value_duree.set("Durée")
entree_duree = tk.Entry(subwindow_2, textvariable = value_duree, width = 20)
entree_duree.pack()

# Position initiale du robot (x, y, angle x->y)
label_position = tk.Label(subwindow_2, text = 'Position initiale du robot', fg = 'navy')
label_position.pack()
# angle
value_angle = tk.IntVar()
value_angle.set("angle")
entree_angle = tk.Entry(subwindow_2, textvariable = value_angle, width = 10)
entree_angle.pack(side = tk.RIGHT)
# y
value_y = tk.IntVar()
value_y.set("y")
entree_y = tk.Entry(subwindow_2, textvariable = value_y, width = 10)
entree_y.pack(side = tk.RIGHT)
# x
value_x = tk.IntVar()
value_x.set("x")
entree_x = tk.Entry(subwindow_2, textvariable = value_x, width = 10)
entree_x.pack(side = tk.RIGHT)

# troisième sous-fenêtre
label_etapes = tk.Label(subwindow_3, text = "Etapes de la trajectoire", fg = 'navy')
label_etapes.pack()

### Fenëtre contenant les caractéristiques du robot (dimensions, positions des capteurs)
robot_window = tk.Frame(window, borderwidth = 10, relief = tk.GROOVE)
title_robot_window = tk.LabelFrame(robot_window, text = 'Robots', fg = 'navy')
robot_window.pack(side = tk.RIGHT)

## Nom du robot/de la configuration
# Création de la sous-fenêtre
name_window = tk.Frame(robot_window, borderwidth = 10, relief = tk.FLAT)
label_name_window = tk.Label(name_window, text = 'Nom du robot', fg = 'navy')
label_name_window.pack()
name_window.pack()
# Entrée du nom du robot
name_robot = tk.StringVar()
value_name = tk.Entry(name_window, textvariable = name_robot, width = 50)
value_name.pack()

## Nombre de capteurs du robot
# Création de la sous-fenêtre
capt_window = tk.Frame(robot_window, borderwidth = 10, relief = tk.FLAT)
label_capt_window = tk.Label(capt_window, text = 'Nombre de capteurs du robot', fg = 'navy')
label_capt_window.grid(row = 0, column = 1)
capt_window.pack()
# Entrée du nombre de capteurs présents sur le robot
nb_capt_robot = tk.IntVar()
value_capt = tk.Entry(capt_window, textvariable = nb_capt_robot, width = 5)
value_capt.grid(row = 1, column = 0)
# Définition du bouton de validation, son activation ouvre une seconde sous-fenêtre où l'on pourra renseigner les coordonnées des capteurs dans le repère du robot (x, y, angle)
button_capt = tk.Button(capt_window, text = 'Valider', fg = 'navy', command = infos_capteurs)
button_capt.grid(row = 1, column = 1)

## Forme du robot
# Création de la sous-fenêtre
pts_robot_window = tk.Frame(robot_window, borderwidth = 10, relief = tk.FLAT)
label_pts_robot_window = tk.Label(pts_robot_window, text = 'Forme du robot', fg = 'navy')
label_pts_robot_window.grid(row = 0, column = 1)
pts_robot_window.pack()
# Entrée du nombre de points 
nb_pts_robot =  tk.IntVar()
value_pts = tk.Entry(pts_robot_window, textvariable = nb_pts_robot, width = 5)
value_pts.grid(row = 1, column = 0)
# Définition du bouton de validation
button_pts = tk.Button(pts_robot_window, text = 'Valider', fg = 'navy', command = infos_points)
button_pts.grid(row = 1, column = 2)

## Dessin du robot
# Création de la sous-fenêtre
dessin_window = tk.Frame(robot_window, borderwidth = 10, relief = tk.FLAT)
label_dessin_window = tk.Label(dessin_window, text = 'Représentation du robot', fg = 'navy')
label_dessin_window.pack()
dessin_window.pack()
# Définition du bouton pour dessiner
button_dessin = tk.Button(dessin_window, text = 'Dessiner', fg = 'navy', command = dessin)
button_dessin.pack()
# Définition du bouton pour réinitialiser la fenêtre
button_clear = tk.Button(dessin_window, text = 'Effacer', fg = 'navy', command = clear)
button_clear.pack()
# Définition du bouton pour sauvegarder la configuration
button_load = tk.Button(dessin_window, text = 'Charger configuration', fg = 'navy', command = load)
button_load.pack(side = tk.BOTTOM)
# Définition du bouton pour sauvegarder la configuration
button_save = tk.Button(dessin_window, text = 'Sauvegarder configuration', fg = 'navy', command = save)
button_save.pack(side = tk.BOTTOM)

### Fenëtre contenant les coefficients utilisés dans le calcul de la trajectoire (alpha_i, beta, gamma) voir AIDE
coeff_window = tk.Frame(window, borderwidth = 10, relief = tk.GROOVE)
title_coeff_window = tk.LabelFrame(coeff_window, text = "Coefficients utilisés pour l'évitement", fg = 'navy')
coeff_window.pack(side = tk.RIGHT)
# Coefficients répulsion
coeff_alpha_window = tk.Frame(coeff_window, borderwidth = 10, relief = tk.GROOVE)
label_coeff_alpha = tk.Label(coeff_alpha_window, text = "Coefficients associés à chaque capteur", fg = 'navy')
label_coeff_alpha.grid(row = 0, column = 0)
coeff_alpha_window.pack()
# Coefficient attraction
coeff_beta_window = tk.Frame(coeff_window, borderwidth = 10, relief = tk.GROOVE)
label_coeff_beta = tk.Label(coeff_beta_window, text = "Coefficient associé à la force d'attraction", fg = 'navy')
label_coeff_beta.pack()
coeff_beta_window.pack()
vbeta = tk.DoubleVar()
vbeta.set(1.0)
value_beta = tk.Entry(coeff_beta_window, textvariable = vbeta, width = 5)
value_beta.pack()
# Coefficient évitement
coeff_gamma_window = tk.Frame(coeff_window, borderwidth = 10, relief = tk.GROOVE)
label_coeff_gamma = tk.Label(coeff_gamma_window, text = "Coefficient")
label_coeff_gamma.pack()
coeff_gamma_window.pack()
vgamma = tk.DoubleVar()
vgamma.set(1.0)
value_gamma = tk.Entry(coeff_gamma_window, textvariable = vgamma, width = 5)
value_gamma.pack()

window.config(menu = menubar)
window.mainloop()
