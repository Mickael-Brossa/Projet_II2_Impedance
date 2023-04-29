# Projet_II2_Impedance
Projet d'informatique embarquée sur la mesure de condensateur et de résistance en régime harmonique, éffectué sur une carte STM32F746.

# Résumé du banc d'essai

Le banc d'essai est réalisé selon le schéma de la photo TOTO.jpg. Le dispositif en lui-même est capable de mesuré des résistance et des capacité à une incertidude proche de 10 %, et cela en faisant varier une résistance, dite de calibrage, et la fréquence du signal sinusoïdale s'abattant sur l'impédance à mesurer. Cette variation de résistance et de fréquence est réalisé par la carte automatiquement, à une vitesse de 2 seconde de variation de la résistance ou de la fréquence. La fréquence du signal sinusoïdale est modifié intrinséquement par la carte, et la résistance de calibrage est modifié par un multiplexeur, controlée par deux pins de la carte.
Les fréquences utilisé dans ce système sont 50 Hz, 250 Hz, 500 Hz, 1 kHz, 2.5 kHz et 5 kHz , et la résistance de calibrage peut prendre quatre valeurs différentes : 3.3 kOhm, 10 kOhm, 33 kOhm et 100 kOhm. Le dispositif est capable de mesuuré des valeurs de résistance allant de 1 kOhm à 600 kOhm et des condensateur de charge variant de 1 nF à 1 uF. Il possible de dépasser cette plage de fonctionnement, mais le système est rapidement dépasser, où donne des résultats trop imprécis.

# Explication du fichier freertos.c

Ce programme contient l'ensemble des tâches permettant le bon fonctionnement du programme. Il y a un total de 6 tâches dans le programme ( la tache defautTask n'est pas comptée) et toute ces tâches sont périodiques, leur priorités sont identique, et leur espace mémoire (pile) font toute la même taille (128 o) à l'exceptiion de la tache affichage (1024 o) car cette dernière doit gérer des variables de plus grande taille, qui servent à l'affichage sur l'écran LCD.

- Tâche affichage : Cette tâche permet d'afficher toutes les informations utiles relatées à la mesure de la capacité ou de la résistance, c'est-à-dire la tension crête au borne de l'impédance à mesurer, si la plage de fonctionnement est dépassée, si le système est encore en phase de modifier la résistance ou la fréquence, et les possible valeurs de résistances ou de capacités mesurée. Cette tâche à une période de 300 ms, ce qui permet de ne pas rafraichir constament l'écran, et donc d'alleger l'utilisation du CPU.

- Tâche Mesure_crete_1 : Cette tâche permet d'obtenir le mot binaire image de la tension d'entrée du CAN 1, qui est proche de la tension crête du signal sinusoïdale au borne de l'impédance à mesurer. Elle permet aussi de déterminer si l'on est en sur-régime (tension trop élever pour effecteur une mesure précise) ou en sous-régime (tension trop faible pour effecteur une mesure précise). Cette tâche à une période de 100 ms, ce qui permet de récuperer assez vite un ensemble de valeur du mot binaire image de la tension crête.

- Tâche Choix_mesure : Elle permet de choisir si l'on désire mesurer une résistance ou une capaciter. Pour mesurer une capaciter, il faut cliquer à gauche de l'écran et pour mesurer une résistance, il faut cliquer à droite de l'écran. La période de cette tâche est de 100 ms, ce qui permet un changement de mesure assez rapide dès lors que l'on a appuyer sur l'écran.

- Tâche Modif_freq_res : Cette tâche à pour utiliter de modifier la fréquence du signal sinusoïdale produit par le DMA en sortie du CNA X1, et aussi de changer le calibre des résistances en exploitant les sorties GPIO 13 et 14 de la carte. La période de cette tâche est de 400 ms, ce qui permet de changer assez vite la fréquence ou la résistance du circuit, sans pour autant être trop rapide, car cela peut faire dysfonctionner le DMA.

- Tâche Ges_freq_res : Cette tâche permet de décider quel changement de fréquence ou de résistance faut-t'il choisir connaissant le régime en cours (sur-régime ou sous-régime) et la fréquence et résistance actuellement utilisé. La période de cette tâche est assez longue (3 sec), car cela permet de ne pas changer trop rapidement le calibre de résistance ou de fréquence, et donc d'éviter une possible oscillation de changement de calibre.

- Tâche Calcul_val : Cette tâche permet de calculer une estimation de l'impédance mise sur le banc de mesure, à partir de la connaissance de la résistance de calibrage, la fréquence des signaux de tension, et le mot binaire image de la tension en entrée du CAN 1. La période de cette tâche est de 1 sec, ce qui est assez long, mais cela permet d'effectuer le calcul des valeurs d'impédance sans causer une possible famine.

Il existe egalement des fil d'attentes qui permettent de faire le lien entre les différentes tâche du programme. Il y a un total de 6 listes d'attentes différentes. Ces listes d'attentes sont principalement utilisé pour déclencher un fonctionnement particulier des tâches du programme.

- Liste d'attente Send_aff_vcc1 : cette liste d'attente permet à la tâche Mesure_crete_1 d'envoyer le mot binaire image de la tension crête en entrée du CAN 1 à la tâche affichage pour que cette dernière puisse l'afficher dès que Mesure_crete_1 capture une valeur.

- Liste d'attente Send_aff_vcc2 : cette liste d'attente n'est plus utilisé, mais elle avait pour but, à l'époque où la tâche Mesure_crete_2 existé, de faire pareil que la file d'attente Send_aff_vcc1.

- Liste d'attente Queue_freq et Queue_res : ces listes d'attentes permettent à la tâche Ges_freq_res d'envoyer la modification de la résistance ou de la fréquence à la tâche Modif_freq_res

- Liste d'attente Queue_cond_est et Queue_res_est : ces listes d'attentes permettent à la tâche Calcul_val d'envoyer la valeur de la résistance ou de la capacité à la tâche affichage pour qe cette dernière puisse les afficher. 






