# FootBot Diffusion - Stratégie Alternative d'Évitement d'Obstacles

## Présentation
Ce projet modifie le contrôleur `footbot_diffusion` dans ARGoS afin d'implémenter une stratégie améliorée d'évitement d'obstacles. Le contrôleur par défaut suit un modèle de diffusion simple, tandis que cette version introduit une approche plus structurée et réactive.

## Stratégie Utilisée

### 1. Détection des Obstacles et Mesure des Distances
- Les lectures des capteurs de proximité sont traitées pour calculer la distance minimale aux obstacles situés devant le robot.
- Une transformation est appliquée afin de convertir ces valeurs en distances réelles selon la formule :  
  $$d = d_{\text{max}} - d_{\text{max}} \times V_{\text{capteur}}$$
  
  où `d_max` est la portée maximale du capteur et `V_capteur` est la valeur normalisée fournie par le capteur (entre 0 et 1).

### 2. Contrôle Adaptatif de la Vitesse
- Le robot ajuste sa vitesse de déplacement en fonction de la distance à l'obstacle, en utilisant une **fonction exponentielle** :  
  $$V_{\text{avance}} = V_{\text{max}} \times \left(1 - e^{-k \cdot (d - d_{\text{seuil}})} \right)$$  
  où `k` est un facteur de réglage et `d_seuil` est la distance minimale avant de déclencher l'évitement.

### 3. Évitement Réactif des Obstacles
- Si un obstacle est **trop proche (≤ 20 cm)**, le robot effectue une **rotation contrôlée de 90°**.
- Pour assurer une rotation précise, une **boussole** est utilisée pour obtenir le cap actuel du robot.
- L’erreur d’orientation est calculée à l’aide de la fonction **sawtooth** :  
  $$\theta_{\text{erreur}} = \arctan2(\sin(\theta_{\text{cible}} - \theta_{\text{actuel}}), \cos(\theta_{\text{cible}} - \theta_{\text{actuel}}))$$
- La vitesse de rotation est ajustée exponentiellement :  
  $$V_{\text{gauche}} = V_{\text{max}} \times \left(1 - e^{-k \cdot \theta_{\text{erreur}}} \right)$$   
  $$V_{\text{droite}} = -V_{\text{max}} \times \left(1 - e^{-k \cdot \theta_{\text{erreur}}} \right)$$  
- Une fois la rotation terminée, le robot reprend son mouvement normal.

### 4. Désactivation Temporaire des Capteurs Pendant la Rotation
- Lors de la phase de rotation d’évitement, les capteurs de proximité sont temporairement désactivés.
- Cette décision a été prise afin d'éviter que de nouvelles mesures pendant la rotation n’altèrent l’exécution du mouvement.
- En désactivant temporairement ces capteurs, on assure une **exécution stable et prévisible** du comportement d’évitement.
- La simulation a été testée **avec et sans cette désactivation** afin d’évaluer son efficacité.

## Justification de l'Utilisation de la Boussole
- Une **boussole** a été ajoutée pour mesurer le cap du robot en temps réel et assurer une rotation précise.
- Une alternative aurait été **l'odométrie**, qui mesure la rotation des roues pour estimer l'orientation.
- Cependant, l'odométrie est moins précise sur de longues distances en raison de l'accumulation d'erreurs de glissement des roues.

## Amélioration Potentielle : Zones de Sécurité
Une amélioration possible serait de définir **deux zones** :
- **Zone sûre** : aucun obstacle détecté, déplacement libre.
- **Zone non sûre** : un obstacle est détecté et une correction de trajectoire est appliquée.

Plutôt que d'effectuer une rotation fixe de 90°, une approche plus optimisée consisterait à **ajuster dynamiquement le cap** en fonction du plus grand espace libre détecté.

**Toutefois, cette amélioration n'a pas été implémentée afin de respecter la contrainte de simplicité demandée (20-25 lignes de code).**

## Conclusion

Cette approche améliore considérablement l'évitement des obstacles en introduisant une estimation précise des distances, un contrôle adaptatif de la vitesse, et une orientation fiable grâce à la boussole. Le robot est ainsi plus réactif, fluide et efficace dans ses déplacements.
