# 🎓 Diplomová práca: Riadenie mobilnej robotickej platformy JetBot počítačom Jetson Nano

## 📘 O projekte

Tento projekt je súčasťou diplomovej práce, ktorej cieľom je **preskúmať možnosti riadenia mobilnej robotickej platformy JetBot pomocou počítača NVIDIA Jetson Nano**. Zároveň skúma možnosti využitia prostredia **ROS (Robot Operating System)** a jeho simulácií v **Gazebo** na návrh, testovanie a implementáciu algoritmov riadenia mobilného robota. Výsledkom je implementácia daných súvislostí na robotovi JetBot.

## 🎯 Ciele práce

- ✅ Preskúmať možnosti JetBot platformy a jej komponentov.  
- ✅ Vytvoriť realistickú simuláciu robota JetBot v prostredí **Gazebo Harmony**.  
- ✅ Implementovať robotické algoritmy v **ROS2 Jazzy** a vizualizovať ich v **RViz2**.  
- ✅ Nasadiť vytvorené algoritmy priamo na reálny JetBot s ROS1 (ROS2) na **Jetson Nano**.  
- ✅ Vyhodnotiť správanie a efektivitu algoritmov v simulácii a na reálnom zariadení.
- ✅ Odtestovať tieto skutočnosti na moiblnej robotickej platforme JetBot 

## 🧠 Použité technológie

- **JetBot ROS AI Kit** od Waveshare  
- **NVIDIA Jetson Nano** od spoločnosti NVIDIA (Ubuntu 18.04.6)  
- **ROS1 Noetic** (na robotovi JetBot)  
- **ROS2 Jazzy** (pre simulácie)  
- **Gazebo 8 Harmony** (simulačné prostredie)  
- **RViz2** (vizualizácia)   

## 📁 Štruktúra projektu

```
ros2_ws/
├── build_project.sh          # Skript určený na build projektu
├── moje_build_logs/          # Mnou vytvorený priečinok na buildovacie logy
├── README.md                 # Dokumentácia projektu 
└── src/                      # Zdrojové ROS2 balíky
    ├── jetbot_description/   # Balík s popisom robota (URDF/SDF modely, RViz konfigurácia...)
    │   ├── config/               # - 
    │   ├── jetbot_description/   # hlavný modul
    │   ├── launch/               # Launch súbor pre spustenie robota v RViz
    │   ├── LICENSE               # Licencia balíka 
    │   ├── models/               # SDF/URDF modely robota a jeho častí
    │   ├── package.xml           # ROS2 metadáta o balíku
    │   ├── resource/             # Súbory na registráciu ROS2 balíka
    │   ├── rviz/                 # RViz konfiguračný súbor (.rviz) rozhrania
    │   ├── setup.cfg             # Nastavenia inštalácie cez ament_python
    │   ├── setup.py              # Inštalačný skript balíka
    │   └── test/                 # Testovacie súbory vytvorené samotným balíkom
    │
    └── jetbot_gazebo/        # Balík pre simuláciu v Gazebo a implementáciu algoritmov
        ├── config/               # YAML konfigurácie 
        ├── jetbot_gazebo/        # hlavný modul
        ├── launch/               # Launch súbory pre spustenie simulácie
        ├── LICENSE               # Licencia balíka 
        ├── models/               # SDF modely robota a jeho komponentov (kamera, LiDAR, antény…)
        ├── package.xml           # ROS2 metadáta o balíku
        ├── resource/             # Súbory na registráciu ROS2 balíka
        ├── setup.cfg             # Nastavenia inštalácie cez ament_python
        ├── setup.py              # Inštalačný skript balíka
        ├── test/                 # Testovacie súbory vytvorené samotným balíkom
        └── worlds/               # Simulačné svety pre Gazebo (napr. test_world.sdf)
```

## 🐍 Implementované algoritmy

Všetky tieto algoritmy sú implementované ako samostatné ROS2 nody v jazyku **Python** a spúšťané cez `jetbot_gazebo.launch.py`:

- `path_publisher.py` – Zaznamenávanie pohybu robota vo svete
- `obstacle_stop.py` – Zastavenie pri prekážke  
- `obstacle_avoidance.py` – Obchádzanie prekážky (otáčanie, pohyb vpred)  
- ~~`run_to_goal.py`~~ – Navigácia k cieľu pomocou stavového automatu
- ~~`wall_following.py`~~ – Sledovanie steny  
- ~~`bug0.py`~~ – Navigácia pomocou Bug 0 algoritmu  
- ~~`bug1.py`~~ – Navigácia pomocou Bug 1 algoritmu
- ďalšie algoritmy 🐣

> **Poznámka**: V priebehu implementovania môžu pribudnúť nové alogirtmy 🚧
> 
> **Poznámka**: Prečiarknuté algoritmy označujú 🚧 - *work in progress*

## 🤖 Simulácia

- **Model robota**: Obsahuje hlavné komponenty, LIDAR, kameru, IMU, stereo, antény, batériu a kontaktné senzory.  
- **Svet**: Definovaný vo vlastnom `.sdf` súbore (`test_world.sdf`).  
- **Ovládanie**: Implementované cez `ros2_control` s pluginom `diffdrive/DiffDrive` a kontrolérom `diff_drive_controller`.  
- **Ovládanie**: robota cez ros2_control **🛠️**
## 🗺️ Spúšťanie simulácie

```
source install/setup.bash
# Gazebo
ros2 launch jetbot_gazebo jetbot_gazebo.launch.py
# Rviz
ros2 launch jetbot_description jetbot_rviz.launch.py
```

Každý algoritmus je možné spustiť pomocou prepínača alebo ako samostatný node v rámci launch súboru.

## 📷 Vizualizácia

Pomocou **RViz2** je možné vizualizovať:

- Model robota  
- Dáta zo senzorov (LaserScan, kamera, odometria...)  
- Trajektóriu a pohyb robota  
- Navigačné ciele a spracované prekážky  

## 🛠️ Inštalácia

1. Klonuj repozitár:
```
git clone https://github.com/Marek21SK/ros2_ws.git
```

2. Zostav workspace:
```
cd ros2_ws
./build_project.sh
```
- Projekt bol vytváraný ako ROS 2 balík typu **ament_python**, ktorý umožňuje spúšťanie Python skriptov ako ROS nodov a ich integráciu do launch súborov.

## 📝 Odkazy

- [Waveshare JetBot ROS AI Kit](https://www.waveshare.com/wiki/JetBot_ROS_AI_Kit)  
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html)  
- [Gazebo Sim (Harmony)](https://gazebosim.org/docs/latest/getstarted/)  
- [Jetson Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)  

---

> 💡 **Tento projekt predstavuje prepojenie medzi simuláciou a reálnym riadením robota, čo umožňuje bezpečné testovanie a nasadenie algoritmov v reálnom prostredí.**
> 