![Bannière](./Useful_Resources/Images_ReadME/banner.png)

# Projet_Industriel_2026

## Description du projet - Plateforme de mobilité autonome pilotée par l'IA

Ce projet porte sur le développement d'une plateforme de mobilité entièrement autonome, conçue selon une approche d'ingénierie industrielle et fortement centrée sur l'intelligence artificielle.

L'objectif est de concevoir un système embarqué intelligent capable de percevoir son environnement, de comprendre les contextes spatiaux et de naviguer de manière autonome en utilisant des méthodologies d'IA modernes.  
Le travail met l'accent sur la **prise de décision autonome**, la **perception de l'environnement**, le **contrôle en temps réel** et la **fiabilité du système**, reflétant les défis rencontrés en robotique avancée et dans les applications de mobilité intelligente.

La plateforme intègre :

- Un **Raspberry Pi** exécutant la logique IA de haut niveau et les pipelines de perception en **Python**,  
- Un **microcontrôleur STM32** assurant le contrôle temps réel et la gestion des actionneurs en **C++**,  
- Une architecture matérielle modulaire incluant LiDAR, caméra, commande moteur et télémétrie.

### Intelligence Artificielle et Perception  
- Vision par ordinateur en temps réel pour l'extraction de caractéristiques et la compréhension spatiale  
- Pipelines d'apprentissage automatique pour la perception, la prédiction et le contrôle autonome  
- Apprentissage par renforcement et imitation pour des comportements de navigation adaptatifs  
- Fusion de capteurs (LiDAR + caméra + télémétrie embarquée)  

### Intelligence Embarquée et Contrôle Temps Réel  
- Boucles de contrôle déterministes pour la direction et la propulsion  
- Acquisition et décodage des capteurs à faible latence  
- Firmware STM32 garantissant un timing constant et une action fiable  
- Modélisation dynamique pour caractériser le rayon de braquage, les contraintes et les budgets de latence  

### Architecture Système et Intégration  
- Architecture matérielle modulaire pour la détection, l'actionnement et le calcul  
- Intégration de LiDAR, modules caméra, contrôleurs de servomoteurs et processeurs embarqués  
- Cartographie spatiale via reconstruction basée sur LiDAR  
- Tests et validations itératifs pour un comportement robuste en conditions réelles  

Cette plateforme sert d'environnement réaliste pour développer et évaluer des techniques d'IA et d'autonomie embarquée de niveau industriel.
