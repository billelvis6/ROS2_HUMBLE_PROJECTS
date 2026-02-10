# Projet 1 - Classe Robot 
 Contexte et objectifs
L’objectif de ce projet est de mettre en pratique les principes de la programmation orientée objet (POO) en C++ à travers la conception d’un système de gestion de différents types de robots.

Le système permet notamment de :

Créer plusieurs types de robots avec des comportements spécifiques.
Simuler leurs mouvements.
Implémenter des fonctionnalités génériques : héritage, polymorphisme, surcharge, encapsulation, etc.
Appliquer les bonnes pratiques de conception logicielle (structure modulaire, classes bien définies, etc.).
2. Architecture générale
Le système est composé de trois classes principales, dont une classe mère : robot.

a. robot(Classe mère)
Il s'agit d'une classe générique représentant un robot. Elle regroupe les attributs et comportements communs à tous les types de robots.

- Attributs principaux : ID du robot, Position 2D : coordonnées (x, y), État : ON ou OFF

- Fonctionnalités : constructeurs / destructeurs, méthodes de déplacement (virtuelles), calcul de la distance à l’origine, méthodes d’affichage

# b. deliveryRobot (Classe fille)
Il s'agit d'un robot de livraison, dérivé de robot.

- Fonctionnalités spécifiques : livrer des entités vers des zones prédéfinies (zone1, zone2), gèrer le nombre d'entités à livrer, implémenter un déplacement basé sur la zone

# c. travellingRobot (Classe fille)
Il s'agit d'un robot qui se déplace librement, dérivé de robot.

- Fonctionnalité spécifique : se déplacer dans une direction donnée(avant, arrière, gauche, droite)

# d. armRobot (Classe fille)
Il s'agit d'un bras robotique, dérivé de robot.

- Fonctionnalité spécifique : effectuer une rotation dans une direction

# 3. Concept de POO utilisés
Encapsulation : Attributs privés, accès par getters/setters
Héritage : Les classes deliveryRobot et travellingRobot héritent de robot
Polymorphisme : Méthodes virtuelles (move, getNameOfRobot)
Surcharge : Constructeurs et opérateurs d’affectation

# 4. Diagramme UML
<img width="1200" height="600" alt="image" src="https://github.com/user-attachments/assets/7adc5ecb-1d2a-4c74-a03a-a037583b7261" />

# 5. Tests réalisés
Création d’objets de chaque classe
Simulation des déplacements (changement de zone / direction)
Vérification des distances depuis l’origine
Test du polymorphisme avec des pointeurs de type robot*

# 6. Résultats obtenus
Chaque type de robot fonctionne indépendamment
Les mouvements sont simulés correctement selon la logique métier
Le code est bien structuré, modulaire, réutilisable et extensible
Les bonnes pratiques C++ sont respectées (gestion mémoire, encapsulation, etc.)

# 7. Limites et perspectives
Limitations :
Les déplacements sont simulés mais pas visualisés graphiquement.
Le système ne gère pas d’environnement réel ou physique
Améliorations possibles :
Ajout d’une interface graphique (SFML, Qt)
Implémentation de logs ou de fichiers de tracking
Extension à d’autres types de robots (volants, marins, etc.)
Gestion d’obstacles, de cartes ou de missions

# 8. Conclusion
Ce projet nous a permis de :

Appliquer les concepts fondamentaux de la programmation orientée objet
Comprendre l’importance de la conception logicielle modulaire
Développer un système cohérent, évolutif et structuré en C++
Il constitue une base solide pour des projets plus complexes, comme des systèmes embarqués dans des robots réels ou des simulations d’intelligence artificielle mobile.

