#  Hexapod ROS 2 Controller & Kinematic Engine

Projet de développement d'un robot hexapode autonome biomimétique à **18 degrés de liberté (18 DOF)** sous **ROS 2 (C++23)** et **ESP32**. 

Ce dépôt contient la chaîne complète de conception : modélisation CAO SolidWorks, calculs de cinématique (FK/IK), génération de trajectoires par courbes de Bézier, contrôle de démarches (tripode, crabe) et nœuds de contrôle ROS 2.

---

## 📑 Sommaire
1. [Conception CAO & Architecture Matérielle](#1-conception-cao--architecture-matérielle)
2. [Modélisation Cinématique (Kinematic Engine)](#2-modélisation-cinématique-kinematic-engine)
3. [Génération de Trajectoires & Démarches](#3-génération-de-trajectoires--démarches)
4. [Architecture Logicielle ROS 2 & Embarqué](#4-architecture-logicielle-ros-2--embarqué)
5. [Installation et Lancement](#5-installation-et-lancement)

---

## 1. Conception CAO & Architecture Matérielle

### Design SolidWorks & Prototypage
* **Modélisation 3D :** Ensemble de la structure (châssis central, articulations Coxa, Femur, Tibia) conçu sous **SolidWorks** en optimisant les rapports masse/rigidité.
* **Fabrication :** Impression 3D FDM sur **Bambu Lab A1 Mini** (Tolérances d'assemblage serrées, remplissage adapté aux contraintes mécaniques).

### Système Embarqué & Énergie
* **Actuateurs :** 18x Servomoteurs haute résistance **MG996R**.
* **Contrôle PWM :** 2x Cartes **PCA9685** (16 canaux I2C chacune) pilotées par un microcontrôleur **ESP32**.
* **Alimentation :** Batterie LiPo 3S (11.1V - 12.6V).
* **Régulation de Puissance :** Module Buck **XL4016** abaissant la tension à $5.5\text{V}$ continu avec une capacité en courant suffisante pour encaisser les pics de consommation des 18 servos simultanés.

---

## 2. Modélisation Cinématique (Kinematic Engine)

Chaque patte $i$ du robot est considérée comme un bras manipulateur série à 3 liaisons pivot réparties dans l'espace : **Coxa ($\theta_1$)**, **Femur ($\theta_2$)**, et **Tibia ($\theta_3$)**.
