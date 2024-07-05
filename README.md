# Projet_GyroPlane
 Projet GyroPlane STM32 ITII ISEN Maxime FALLABRINO Julien COLARD

 Un projet consistant à reproduire un "cockpit" d'avion avec l'utilisation de différents capteurs :

 - Pression et Température : pour calcul d'altitude
 - Accélèromètre : pour calcul angle de rotation
 - Gyroscope : pour vitesse de rotation

Nous affichons toutes ces données récupérées et calculées sur le terminal Teraterm. 
L'altitude est tracée sur le graphe ITM datatrace du mode debug.
La pression est écrite sur l'afficheur 7 segments.

De plus, le potentionmètre est utilisé pour faire varier la vitesse de notre moteur et ainsi simuler l'allumage du moteur pour permettre l'enclenchement du programme.
Aussi,j'utilise les leds pour démontrer l'angle de rotation ( 1 led supplémentaire tous les 10°) et ainsi allumer finalement les 8 leds lorsqu'on arrive à 80°. 

A 80° de rotation, on a toutes les leds qui clignotent + le buzzer qui fait l'alarme + le moteur qui tourne à fond, tout ça afin de simuler une zone de décorchage et ainsi l'empêcher. Tout ça se stoppe dès lors que l'angle devient inférieur à 80°.


Fonctionnement : 

1ère étape : faire évoluer le potentionmètre jusqu'à une valeur de vitesse moteur minimum + appuyer sur le BP2 pour lancer le programme  
2ème étape : utiliser la carte à bon escient


Nécessaires : 

- Installer le driver : MAX7219 pour l'afficheur 7 segments
- Initialiser correctement Teraterm : Sur le port série correspondant + vitesse de transmission 115200 Bauds (si Huart2 configurer comme ça)
- Initialiser X-CUBE-MEMS1 pour que les capteurs fonctionnent 







