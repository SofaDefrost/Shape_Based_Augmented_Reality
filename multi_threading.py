import threading
import time

# Fonction à exécuter dans le thread
def fonction1():
    i=1
    while True:
        print("Fonction 1 début")
        time.sleep(5)  # Simule une tâche longue
        resultat_partage.append(str(i))
        print("Fonction 1 fin")
        i+=1

# Deuxième fonction à exécuter dans le thread principal
def fonction2():
    while True :
        print("Fonction 2 début")
        time.sleep(3)  # Simule une autre tâche un peu moin longue
        print("Résultat reçu de la fonction 1 :", resultat_partage)
        print("Fonction 2 fin")

# Liste partagée pour stocker le résultat
global resultat_partage

resultat_partage = ["0"]

# Créer un thread pour la première fonction
thread_fonction1 = threading.Thread(target=fonction1)

# Démarrer le thread de la première fonction
thread_fonction1.start()

fonction2()

# Attendre que le thread de la première fonction se termine
thread_fonction1.join()

print("Fin du programme")
