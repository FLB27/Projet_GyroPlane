# Projet_GyroPlane
 Projet GyroPlane STM32 ITII ISEN P21 Maxime FALLABRINO Julien COLARD

 Un projet consistant à reproduire un "système de navigation" d'avion avec l'utilisation de différents capteurs :

 - Pression et Température : pour calcul d'altitude
 - Accélèromètre : pour calcul angle de rotation (autour de l'axe x et y uniquement)
 - Gyroscope : pour mesure de la vitesse de rotation

Nous affichons toutes les données récupérées et calculées sur le terminal Teraterm. 
L'altitude est tracée sur le graphe ITM datatrace du mode debug.
La pression est écrite sur l'afficheur 7 segments.

De plus, le potentionmètre est utilisé pour faire varier la vitesse de notre moteur et ainsi simuler l'allumage du moteur pour permettre l'enclenchement du programme.

Aussi,j'utilise les leds pour préciser l'angle de rotation autour de x et/ou y ( 1 led supplémentaire tous les 10°) et ainsi allumer finalement les 8 leds lorsque nous arrivons à 80° ou -80°. 

A 80° ou -80° de rotation, nous avons toutes les leds qui clignotent + le buzzer qui fait l'alarme + le moteur qui tourne à fond, tout cela afin de simuler une zone de décrochage . Tout ça se stoppe dès lors que l'angle devient inférieur à 80° ou -80°. Autrement, le buzzer peut-être coupé par une pression sur le BP1.

*****************
Prérequis : 

- Installer le driver : "display" permettant d'obtenir la bibliothèque MAX7219 pour l'afficheur 7 segments
- Initialiser correctement Teraterm : Sur le port série correspondant + vitesse de transmission 115200 Bauds (si Huart2 configuré comme cela)
- Initialiser X-CUBE-MEMS1 : Dans l'IOC télécharger et initialiser ce package afin que les différents capteurs puissent fonctionner. 
******************

Fonctionnement : 

1ère étape : faire évoluer le potentionmètre jusqu'à une valeur ADCValue(visible sur le terminal) minimum égale à 1000, permettant d'atteindre la vitesse moteur minimum pour l'allumage des équipements + appuyer sur le BP2 pour lancer le programme  

2ème étape : utiliser la carte à bon escient (faites varier l'angle de rotation selon l'axe x et y + faites varier la vitesse du moteur avec le potentiomètre,etc.)

*******************








