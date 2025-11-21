# Firmware Description
📦 Firmware overview for Projet_Industriel_2026

## Overview
This page describes the firmware architecture, build and flash workflow, and practical tips for development and debugging.

## Architecture (high level)
```mermaid
graph TD
    %% Définition des styles (Styles)
    classDef appLayer fill:#ffcccc,stroke:#b30000,stroke-width:2px;
    classDef rteLayer fill:#ccffcc,stroke:#006600,stroke-width:2px;
    classDef bswLayer fill:#ccccff,stroke:#000066,stroke-width:2px;
    classDef hwLayer fill:#e0e0e0,stroke:#333333,stroke-width:2px,shape:cylinder;

    %% ==========================
    %% 1. Couche Application
    %% ==========================
    subgraph "Couche Application (Application Layer) - Nœuds ROS"
        direction TB
        Perception(SWC Perception<br>Caméra & Lidar):::appLayer
        Planning(SWC Planification<br>Trajectoire & Décision):::appLayer
        Control(SWC Contrôle<br>Latéral & Longitudinal):::appLayer
    end

    %% ==========================
    %% 2. Environnement d'Exécution (RTE)
    %% ==========================
    subgraph "Environnement d'Exécution (RTE) - Middleware ROS"
        RTE(ROS 2 DDS / Topics & Services<br>Bus Virtuel):::rteLayer
    end

    %% ==========================
    %% 3. Logiciel de Base (BSW)
    %% ==========================
    subgraph "Logiciel de Base (BSW) - Pilotes & Firmware"
        direction TB
        
        subgraph "Abstraction Matérielle E/S"
            Lidar_Drv(Pilote Lidar<br>Paquet ROS):::bswLayer
            Cam_Drv(Pilote Caméra<br>GStreamer/V4L2):::bswLayer
            Serial_Drv(Pilote Série<br>USB-UART):::bswLayer
        end
        
        subgraph "Firmware Microcontrôleur (STM32)"
            PWM_Gen(Génération PWM):::bswLayer
            IMU_Read(Lecture IMU):::bswLayer
            Safety_Mon(Surveillance Sécurité / Arrêt d'Urgence):::bswLayer
        end
    end

    %% ==========================
    %% 4. Couche Matérielle
    %% ==========================
    subgraph "Couche Matérielle (Hardware)"
        Jetson(Jetson Nano / RPi):::hwLayer
        STM32(STM32 Nucleo):::hwLayer
        Sensors(Capteurs : Lidar & Caméra):::hwLayer
        Actuators(Actionneurs : Moteur & Servo):::hwLayer
    end

    %% ==========================
    %% Connexions (Relations)
    %% ==========================
    
    %% APP to RTE
    Perception <--> RTE
    Planning <--> RTE
    Control <--> RTE

    %% RTE to BSW
    RTE <--> Lidar_Drv
    RTE <--> Cam_Drv
    RTE <--> Serial_Drv

    %% BSW Internal
    Serial_Drv <==>|Protocole UART| PWM_Gen
    Serial_Drv <==>|Protocole UART| IMU_Read

    %% BSW to Hardware
    Lidar_Drv --- Sensors
    Cam_Drv --- Sensors
    PWM_Gen --- STM32
    STM32 --- Actuators
    Safety_Mon --- STM32
```