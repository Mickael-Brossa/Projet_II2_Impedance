# Projet_II2_Impedance
Projet d'informatique embarquée sur la mesure de condensateur et de résistance en régime harmonique, éffectué sur une carte STM32F746.

# Résumé sur le banc d'essai

Le banc d'essai est réalisé selon le schéma de la photo TOTO.jpg. Le dispositif en lui-même est capable de mesuré des résistance et des capacité à une incertidude proche de 10 %, et cela en faisant varier une résistance, dite de calibrage, et la fréquence du signal sinusoïdale s'abattant sur l'impédance à mesurer. Cette variation de résistance et de fréquence est réalisé par la carte automatiquement, à une vitesse de 2 seconde de variation de la résistance ou de la fréquence. La fréquence du signal sinusoïdale est modifié intrinséquement par la carte, et la résistance de calibrage est modifié par un multiplexeur, controlée par deux pins de la carte.
Les fréquences utilisé dans ce système sont 50 Hz, 250 Hz, 500 Hz, 1 kHz, 2.5 kHz et 5 kHz , et la résistance de calibrage peut prendre quatre valeurs différentes : 3.3 kOhm, 10 kOhm, 33 kOhm et 100 kOhm. Le dispositif est capable de mesuuré des valeurs de résistance allant de 1 kOhm à 600 kOhm et des condensateur de charge variant de 1 nF à 1 uF. Il possible de dépasser cette plage de fonctionnement, mais le système est rapidement dépasser, où donne des résultats trop imprécis.

# Explication du fichier freertos.c

Ce programme contient l'ensemble des tâches permettant le bon fonctionnement du programme. Il y a un total de 6 tâches dans le programme ( la tache defautTask n'est pas comptée) et toute ces tâches sont périodiques, leur priorités sont identique, et leur espace mémoire (pile) font toute la même taille (128 o) à l'exceptiion de la tache affichage (1024 o) car cette dernière doit gérer des variables de plus grande taille, qui servent à l'affichage sur l'écran LCD.

- Tâche affichage : Cette tâche permet d'afficher toutes les informations utiles relatées à la mesure de la capacité ou de la résistance, c'est-à-dire la tension crête au borne de l'impédance à mesurer, si la plage de fonctionnement est dépassée, si le système est encore en phase de modifier la résistance ou la fréquence, et les possible valeurs de résistances ou de capacités mesurée

- Tâche Mesure_crete_1 : Cette tâche permet d'obtenir le mot binaire image de la tension d'entrée du CAN, qui est proche de la tension crête du signal sinusoïdale au borne de l'impédance à mesurer. Elle permet aussi de déterminer si l'on est en sur-régime (tension trop élever pour effecteur une mesure précise) ou en sous-régime (tension trop faible pour effecteur une mesure précise). Cette tâche à une période de 100 ms, ce qui permet de récuperer assez vite un ensemble de valeur du mot binaire image de la tension crête.

- Tâche Choix_mesure : Elle permet de choisir si l'on désire mesurer une résistance ou une capaciter. Pour mesurer une capaciter, il faut cliquer à gauche de l'écran et pour mesurer une résistance, il faut cliquer à droite de l'écran.




