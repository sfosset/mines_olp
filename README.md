# Apprentissage en ligne pour Pioneer
## Desription
Le problème est le suivant : un robot Pioneer se situe dans une certaine
configuration (x, y, theta) dans l'espace et doit rejoindre une cible
(x', y', theta').
Il est commandé par un réseau de neurones qui prend en entrée l'erreur entre
la position réelle et la position cible et donne en sortie la commande sur les
deux roues (q_1, q_2)

## Arborescence
* BackProp_Python_v2.py : le modèle du réseau de neurone utilisé dans
  run.py ainsi que la rétropropagation du gradient associée.
  * Note : Dans le constructeur de NN, on peut modifier l’intervalle des valeurs
  de départ des poids aux lignes où randomizeMatrix est appelée.

* online_trainer.py : c'est dans ce fichier que se situe l'algorithme
  d'apprentissage, ainsi que le calcul du gradient.
* run.py : le fichier qu'il faut lancer pour utiliser l'application. Attention
  à bien le modifier selon les besoins (robot réel, robot simulé)
* simu.ttt : la scène V-REP contenant le modèle du Pioneer
* vrep_pioneer_simulation.py : le modèle du Pioneer simulé
* rdn.py : le modèle du Pioneer réel, contrôlé avec RosAria
* v1.py : une première version de l'apprentissage sur simulateur, non essentiel
  à la simulation
* BackProp_Python.py : le modèle original, qui peut être utilisé pour
  d'autres projets, non essentiel à la simulation

## Utilisation
*Attention* : testé sous python 3.5 uniquement
### Simulation
L'apprentissage sur la simulation se fait avec le simulateur V-REP. La première
chose que vous devez donc faire est de [télécharger](http://www.coppeliarobotics.com/downloads.html)
la version de V-REP correspondant à votre OS (Win, OS X, Linux disponibles).
Dans ce code, on utilise l'API externe de V-REP en python, il va donc falloir
que vous copiez trois fichiers dans ce dossier :
* vrep.py et vrepConst.py que vous trouverez dans les fichiers d'installation
de V-REP sous programming/remoteApiBindings/python/python
* remoteApi.?? dont l'extension dépendra de votre OS (Linux : .so, Win : .dll,
  OS X : .dylib)
que vous trouverez sous programming/remoteApiBindings/lib/lib

Avant de lancer le programme, il faut modifier run.py pour s'assurer que
l'objet `robot` au début du fichier est bien une instance de robot simulé :
```
robot = VrepPioneerSimulation()
```
Il suffit ensuite de lancer le fichier run.py et de suivre les instructions :  
```
$ python run.py
```
Si tout fonctionne correctement, l’utilisateur verra apparaître ‘Connected to
remote API server on…’ puis l’IP du serveur vrep. On peut modifier l’IP et le
port dans le fichier vrep_pioneer_simulation.py.

A la fin de la simulation, les poids de la dernière itération sont enregistrés
dans le fichier `last_w.json`. Attention, celui-ci écrase le précédent fichier
si son nom n'a pas été changé !

### Robot réel
*Si vous êtes déjà familier avec ROS, rospy et RosAria, ou si vous utilisez le programme avec l'ordinateur ainsi que la box dédiés à Minoïde vous pouvez sauter cette section et vous diriger à STOP* `GOTO : STOP`

Dans cette section nous donnons des indications sur l'installation de ROS, RosAria, et sur la configuration nécessaire pour faire fonctionner le robot Pioneer avec un cable USB-SERIE.

___ROS___

Robot Operating System, cette surcouche logicielle s'installe au dessus d'un noyau linux (Version utilisée : hydro, couplée à ubuntu 12.04).
Pour plus d'informations concernant l'installation :
http://wiki.ros.org/ROS/Installation
Le tutoriel est suffisamment détaillé, suivez simplement le déroulé des instructions de la documentation officielle, page ci-dessus et seulement celle-ci, dans le bon ordre. Sauter une étape pour aller plus vite est une mauvaise idée, surtout si vous devez réinstaller ubuntu.

___ROSARIA___

Une fois l'installation terminée, vous aurez certainement configuré un path pour votre espace de travail catkin.
Dans ce même espace, vous devrez installer RosAria (que vous pouvez télécharger avec un gestionnaire de paquets ou à partir des sources officielles sur github ou svn).

```sudo apt-get install ros-<version>-<paquet> ```

(utilisez la touche tab à fur et à mesure que vous cherchez vos paquets pour vérifier que les noms correspondent)
puis compiler vos dossiers sources grâce à la commande catkin_make que vous effectuez au top du dossier src. Par exemple si votre RosAria se trouve dans ~/catkin_ws/src :

```$ cd ~/catkin_ws; catkin_make```

Pour plus d'informations sur RosAria :

http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA

RosAria est l'API qui permet de commander le pioneer, et de recevoir des informations de ses différents capteurs. RosAria n'est qu'un pont compatible ROS qui fait le lien avec la librairie native du pioneer, Aria. Pour l'usage qui est fait du pioneer, vous n'aurez pas à vous occuper d'Aria. Cependant, il faut configurer les ports utilisés.

___Configuration___

A ce stade, vous avez donc configuré dans votre ~/.bashrc une redirection vers votre espace de travail (Conf. le wiki pour l'installation ), installé RosAria dans votre espace de travail catkin, et compilé les sources.


Vous pouvez brancher le cable USB-SERIE sur le pioneer et lancer rosaria avec la commande :
```rosrun rosaria RosAria```
Vérifiez que vous avez les privilèges pour utiliser le port USB. Si ce n'est pas le cas ajoutez votre compte utilisateur linux à la liste des utilisateurs autorisés.
Vous devrez _éventuellement_ configurer le nom du port usb pour RosAria :
- La commande utile pour faire une liste des devices branchés: `ls /dev`
- Pour configurer le nom du port USB à utliser, vous pouvez soit modifier le fichier ~/path to catkin workspace/src/RosAria/RosAria.cpp pour y mettre le port adéquat, par exemple '/dev/ttyUSB0'. Ou alors le passer en paramètre. (Section 1.6 du lien wiki sur RosAria)

___Utilisation de ROS en réseau___

Une dernière section pour l'exploitation de ROS en réseau, le cas d'utilisation que nous avons choisi.
La première chose à savoir est que sur l'ordinateur+routeur Minoïde, tout est déjà configuré pour fonctionner correctement, vous n'avez aucune configuration, installation ou commande à taper pour faire fonctionner RosAria.
Si en revanche vous voulez tout réinstaller, nous donnons quelques informations pratiques qui peuvent vous aider à aller plus vite.

Quelques informations et règles de base :
- ROS est un serveur de gestion de noms, dont la principale utilité est la redirection d'information vers les bons programmes (noeuds), soit de manière asynchrone (à travers des tunnels d'information, appelés topics), on parle alors d'environnement publisher/subscriber. Soit de manière synchrone, via des services (Rosservice), généralement pour changer des paramètres.
-   Il ne peut existe qu'un seul serveur ROS. Celui-ci tourne sur une machine connectée via un réseau local (à travers un routeur qui fait office de serveur DHCP). Les autres machines qui veulent communiquer avec les noeuds/services peuvent le faire sans instancier ROS. En revanche il faut clairement indiquer deux informations :
  1. La première est l'ip du serveur ros, dans le fichier ~/.bashrc, il faut rajouter donc, si 192.168.0.13 est l'ip du linux hébergeant l'instance roscore : ```export ROS_MASTER_URI=http://192.168.0.13:11311``` .
  2. La seconde est l'ip de l'ordinateur qui interroge le serveur ROS. Autrement dit, si vous êtes l'ordinateur 192.168.0.12, il faut rajouter dans votre ~/.bashrc :
  ```export ROS_IP=192.168.0.12``` . Une astuce est de remplacer l'ip écrite en dur par $(hostname -I), à utiliser si votre ip change régulièrement.
- Quel que soit le rôle de l'ordinateur utilisé, il doit avoir ROS installé d'une part, et d'autre part il doit déclarer le ROS_MASTER_URI qui est fixé selon l'ordinateur serveur, ROS_IP qui n'est que l'ip locale propre à chaque ordinateur. Le seul ordinateur où ROS_IP == ROS_MASTER_URI est le serveur.
- De toute évidence, les ordinateurs doivent être connectés au même réseau local.

Pour plus d'information, consulter le wiki officiel qui détaille assez bien l'utilisation complète de ROS en réseau.

`STOP`

L'apprentissage sur le robot réel se fait grâce au contrôleur RosAria

Avant de lancer le programme, il faut modifier run.py pour s'assurer que
l'objet `robot` au début du fichier est bien une instance du controleur du
Pioneer réel via RosAria, autrement dit, cette commande ne doit pas être commentée sur le fichier de lancement :
```
robot = Pioneer(rospy)
```
Une fois que RosAria est lancé, il suffit enfin de lancer le fichier run.py et de suivre les instructions :  
```
$ python run.py
```
A la fin de la simulation, les poids de la dernière itération sont enregistrés
dans le fichier `last_w.json`. Attention, celui-ci écrase le précédent fichier
si son nom n'a pas été changé !

__DEBUGGAGE RAPIDE__

- Il est fort possible que les programmes qui tournent sur l'ordinateur embarqué 'NUC' du pioneer ( et qui sont lancés automatiquement au démarrage ) aient changés. Donc il est possible qu'un comportement inattendu soit du à des programmes qui tournent parallèlement à RosAria.
Une manière simple de résoudre ce problème est de se connecter en ssh à l'ordinateur embarqué :
``` ssh texlab@192.168.0.x ``` (remplacer x par le bon nombre).
Une fois connecté, fermez tous les programmes ROS : ```$(pkill ROS)```, puis lancez uniquement RosAria ( roscore dans un shell, et RosAria dans un autre).

- Un autre souci peut venir du routage réseau. Vérifiez que les ips sont sont correctement référencées, que les ROS_IP et ROS_MASTER_URI correspondent également à ce que vous souhaitez.
