# Hexapode 18 DOF (ESP32)

Un robot hexapode autonome à 18 degrés de liberté, conçu pour se déplacer en milieu instable là où les roues ne passent pas. 

Conçu sous SolidWorks, imprimé en 3D et piloté par un microcontrôleur ESP32.

<img width="825" height="641" alt="Capture d&#39;écran 2026-08-02 155225" src="https://github.com/user-attachments/assets/1ae9c875-4b61-4637-b171-e46fbfd2c9b4" />


---

## 🔩 Matériel & Structure

* **Structure & Chassis :** Pièces conçues sur-mesure et imprimées en 3D sur **Bambu Lab A1 Mini**.
* **Actuateurs :** 18x servomoteurs **MG996R** (3 articulations par patte : Coxa, Femur, Tibia).
* **Contrôle PWM :** 2x cartes **PCA9685** (16 canaux I2C chacune) reliées à l'**ESP32**.
* **Alimentation :** Batterie **LiPo 3S** couplée à un convertisseur Buck **XL4016** (réduit la tension à 5.5V pour encaisser les pics de courant des 18 servos).

<img width="825" height="641" alt="IMG_20260802_143105" src="https://github.com/user-attachments/assets/a366668a-da39-4b24-94c6-26b9f427508b" />

---

## 📐 Cinématique (Kinematics)

Pour simplifier les calculs, les équations sont d'abord appliquées à **une seule patte** (Patte $i$), puis transposées aux autres pattes par changement de repère.

### 1. Cinématique Directe (Forward Kinematics - FK)

La cinématique directe consiste à calculer la position 3D exacte du bout de la patte $(x, y, z)$ à partir des angles mesurés ou appliqués sur chaque servomoteur $(\theta_1, \theta_2, \theta_3)$.

#### À quoi servent les paramètres de Denavit-Hartenberg (DH) ?
Plutôt que de calculer des projections trigonométriques complexes en 3D à la main pour chaque segment, la **convention de Denavit-Hartenberg (DH)** offre un cadre mathématique standardisé. Elle permet de passer du repère d'un joint au repère du joint suivant à l'aide d'une matrice de transformation homogène unique $T$, définie par seulement **4 paramètres** :

* **$a_i$ (Longueur du maillon) :** Distance entre l'axe $Z_{i-1}$ et l'axe $Z_i$, mesurée le long de l'axe $X_i$.
* **$\alpha_i$ (Torsion du maillon) :** Angle d'inclinaison entre l'axe $Z_{i-1}$ et l'axe $Z_i$, mesuré autour de l'axe $X_i$.
* **$d_i$ (Décalage du maillon) :** Distance le long de l'axe $Z_{i-1}$ entre l'origine et l'intersection avec $X_i$ ($0$ sur notre structure).
* **$\theta_i$ (Angle d'articulation) :** Angle de rotation autour de l'axe $Z_{i-1}$ (la variable pilotée par le servo).

#### Comment sont choisis les repères sur la patte ?
La construction des repères $R_0, R_1, R_2, R_3$ suit rigoureusement la convention classique de la main droite :

1. **Axe $Z_i$ (Axe de rotation) :** L'axe $Z_i$ est **toujours aligné avec l'axe de rotation** du joint suivant $(i+1)$.
   * $Z_0$ est vertical : c'est l'axe de rotation de la Coxa (orientation gauche/droite).
   * $Z_1$ est horizontal : c'est l'axe de rotation du Fémur (levée du bras).
   * $Z_2$ est horizontal et parallèle à $Z_1$ : c'est l'axe de rotation du Tibia.
2. **Axe $X_i$ (Axe du segment) :** L'axe $X_i$ est placé le long du segment physique (Coxa $L_1$, Fémur $L_2$, Tibia $L_3$), pointant directement vers le joint suivant.
3. **Axe $Y_i$ (Axe complémentaire) :** Il est déterminé automatiquement par la règle de la main droite pour former un repère direct ($Y_i = Z_i \times X_i$).

<img width="792" height="668" alt="image" src="https://github.com/user-attachments/assets/2c65a576-06ad-4939-859c-dc40bc8c2795" />


#### Tableau des paramètres DH de la patte

| Joint $i$ | $a_i$ | $d_i$ | $\alpha_i$ | Variable $\theta_i$ | Explication géométrique |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **1 (Coxa)** | $L_1$ | $0$ | $90^\circ$ | $\theta_1$ | Le passage de $Z_0$ (vertical) à $Z_1$ (horizontal) impose une torsion $\alpha_1 = +90^\circ$. |
| **2 (Femur)** | $L_2$ | $0$ | $0^\circ$ | $\theta_2$ | Les axes $Z_1$ et $Z_2$ sont parallèles ($\alpha_2 = 0^\circ$). Le mouvement reste coplanaire. |
| **3 (Tibia)** | $L_3$ | $0$ | $0^\circ$ | $\theta_3$ | L'axe $Z_2$ et l'axe fictif $Z_3$ sont parallèles. Le pied se déplace dans le plan du fémur. |

#### Matrice de transformation & Équations finales
En multipliant les matrices de transformation homogènes ${}^{i-1}_{i}T$ associées à chaque ligne du tableau :

$${}^0_3T = {}^0_1T(\theta_1) \cdot {}^1_2T(\theta_2) \cdot {}^2_3T(\theta_3)$$

On extrait la position spatiale $(x, y, z)$ de la pointe du pied dans le repère de la base de la patte ($R_0$) :

$$\begin{cases}
x = (L_1 + L_2 \cos\theta_2 + L_3 \cos(\theta_2 + \theta_3)) \cos\theta_1 \\
y = (L_1 + L_2 \cos\theta_2 + L_3 \cos(\theta_2 + \theta_3)) \sin\theta_1 \\
z = L_2 \sin\theta_2 + L_3 \sin(\theta_2 + \theta_3)
\end{cases}$$

<img width="756" height="544" alt="image" src="https://github.com/user-attachments/assets/30830f92-bb57-4f52-a458-b100238b4594" />



---

### 2. Cinématique Inverse (Inverse Kinematics - IK)
À partir d'une position cible $(X, Y, Z)$ dans le repère de la patte, on cherche les angles $(\theta_1, \theta_2, \theta_3)$ :

<img width="1028" height="651" alt="image" src="https://github.com/user-attachments/assets/d48de7e5-1ab8-458c-92e7-1973bcb1450f" />


1. **Angle de Coxa ($\theta_1$) :**
   $$\theta_1 = \arctan2(Y, X)$$

2. **Projection 2D dans le plan de la patte :**
   $$r_2 = \frac{X}{\cos\theta_1} - L_1, \quad r_1 = \sqrt{Z^2 + r_2^2}$$

3. **Angles de Femur ($\theta_2$) et Tibia ($\theta_3$) (Loi des cosinus) :**
   $$\phi_1 = \arccos\left(\frac{L_3^2 - L_2^2 - r_1^2}{-2 L_2 r_1}\right), \quad \phi_2 = \arctan2(-Z, r_2)$$
   $$\theta_2 = \phi_1 + \phi_2$$
   $$\phi_3 = \arccos\left(\frac{r_1^2 - L_2^2 - L_3^2}{-2 L_2 L_3}\right) \implies \theta_3 = -(\pi - \phi_3)$$

---

## Génération de Trajectoire & Démarches

### 1. Courbes de Bézier (Phase d'envol / Swing)
Pour éviter les secousses lors des pas, la trajectoire en phase d'envol suit une **courbe de Bézier quadratique** contrôlée par 3 points ($P_1$ départ, $P_2$ hauteur de pas, $P_3$ arrivée) :

$$B(t) = (1-t)^2 P_1 + 2(1-t)t P_2 + t^2 P_3 \quad \text{avec } t \in [0, 1]$$

> **[Insérer schéma Bézier / Single Leg Motion]**

### 2. Transposition du repère Châssis vers les Pattes
Pour calculer le mouvement des 6 pattes, on applique une matrice de rotation $ROT(\beta)$ et un décalage physique $X_{start}$ selon l'orientation de chaque patte sur le châssis :

$$\begin{bmatrix} X \\ Y \\ Z \end{bmatrix}_{\text{patte}} = [ROT(\beta)] \times \begin{bmatrix} X \\ Y \\ Z \end{bmatrix}_{\text{châssis}} + \begin{bmatrix} X_{start} \\ 0 \\ 0 \end{bmatrix}$$

> **[Ne pas oublier schéma Repère O-Frame vers Pattes]**

### 3. Modes de Déplacement
* **Marche Tripode (Tripod Gait) :** Alternance de 2 groupes de 3 pattes pour une marche rapide.
* **Marche en Crabe :** Translation latérale ou oblique en orientant la matrice de trajectoire sans tourner le corps.
* **Rotation sur Place :** Mouvement opposé (Drag/Swing) entre les deux groupes de pattes.
* **Virage / Steering (ICC) :** Calcul de trajectoires en arc de cercle autour d'un centre instantané de rotation ($R_{ICC}$).

> **[schéma Steering ICC]**

---

## 💻 Structure du Code ESP32 & Calibrage

### Architecture du Code (Arduino / C++)
# Charger l'environnement
source install/setup.bash
### Table de Mappage Angles Cinématiques -> Impulsions PWM
Chaque servo est calibré pour convertir l'angle théorique de l'IK en valeur PWM ($800\mu s - 2200\mu s$) envoyée aux cartes PCA9685 :

| Articulation | Type Servo | Angle Cinématique | Angle Servo Réel | Plage PWM ($\mu s$) |
| :---: | :---: | :---: | :---: | :---: |
| **Joint 1 (Coxa)** | MG996R | $-90^\circ \text{ à } +90^\circ$ | $0^\circ \text{ à } 180^\circ$ | $800 \text{ à } 2200$ |
| **Joint 2 (Femur)** | MG996R | $-90^\circ \text{ à } +90^\circ$ | $0^\circ \text{ à } 180^\circ$ | $800 \text{ à } 2200$ |
| **Joint 3 (Tibia)** | MG996R | $-180^\circ \text{ à } 0^\circ$ | $0^\circ \text{ à } 180^\circ$ | $800 \text{ à } 2200$ |

> **[Insérer schéma de calibration Servo]**
