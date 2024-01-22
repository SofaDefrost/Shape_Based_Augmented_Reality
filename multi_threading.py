import threading
import time

import functions.icp as f
# Pas très concluant en terme de performance

def fonction(points_source,points_target,angle_x,angle_y,angle_z):
    for i in range(-180,0,20):
        for j in range(-180,0,20):
            for k in range(-180,180,20):
                matrix,cost=f.get_cost_to_align_points(points_source,points_target,j,k,angle_z+i)
                resultat_partage.append([matrix,cost])
    
def fonction2(points_source,points_target,angle_x,angle_y,angle_z):
    for i in range(0,180,20):
        for j in range(0,180,20):
            for k in range(-180,180,20):
                matrix,cost=f.get_cost_to_align_points(points_source,points_target,j,k,angle_z+i)
                resultat_partage.append([matrix,cost])

def fonction3(points_source,points_target,angle_x,angle_y,angle_z):
    for i in range(0,180,20):
        for j in range(180,0,20):
            for k in range(-180,180,20):
                matrix,cost=f.get_cost_to_align_points(points_source,points_target,j,k,angle_z+i)
                resultat_partage.append([matrix,cost])
 
def fonction4(points_source,points_target,angle_x,angle_y,angle_z):
    for i in range(-180,0,20):
        for j in range(0,180,20):
            for k in range(-180,180,20):
                matrix,cost=f.get_cost_to_align_points(points_source,points_target,j,k,angle_z+i)
                resultat_partage.append([matrix,cost])                
def multi_threading(points_source,points_target):
    # Liste partagée pour stocker le résultat
    global resultat_partage
    resultat_partage = []
    
    thread_fonction1 = threading.Thread(target=fonction, args=(points_source,points_target,0,0,0))
    thread_fonction2 = threading.Thread(target=fonction2, args=(points_source,points_target,0,0,0))
    thread_fonction3 = threading.Thread(target=fonction3, args=(points_source,points_target,0,0,0))
    thread_fonction4 = threading.Thread(target=fonction4, args=(points_source,points_target,0,0,0))


    # Démarrer le thread des fonctions

    thread_fonction1.start()
    thread_fonction2.start()
    thread_fonction3.start()
    thread_fonction4.start()

    # Attendre que tout les thread se terminent

    thread_fonction1.join()
    thread_fonction2.join()
    thread_fonction3.join()
    thread_fonction4.join()
