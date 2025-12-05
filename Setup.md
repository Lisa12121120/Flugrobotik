# Beginn
## 1. ROS2 Seite
```
git clone --recurse https://github.com/DynamicSwarms/ds-crazyflies.git
```
---

## 2. MobaXterm (optional?)
Install and launch MobaXterm:

https://mobaxterm.mobatek.net/download.html

---

## 3. Webots installieren
https://cyberbotics.com/#download

---

## 4. Webots welt
```
git clone https://github.com/DynamicSwarms/crazywebotsworld.git
```
---

## 5. Welt in Webots öffnen
---

## 6. Docker starten

vorhande Docker Container
```
docker ps
```

```
docker stop ds-crazyflies-dev
docker rm ds-crazyflies-dev
```

### devcontainer starten
in vs code Extensions 
- devcontainer 
- Docker

F1 drücken (oben in der Zeile) Dev Container: Reopen in Container (ggf. without cache)

jetzt dauerts (10 min oder so) (hoffentlich XD)

### Im Container

Workspace bauen oder so

muss man immer machen wenn man eine neue Node schreibt. müsste eigentlich noch einen effzienteren Weg geben

dauert auch lang (10 min oder so)

nicht in dem Terminal das aufgeht, wenn der Container startet (hatt bei mir eine Warnsymbol)
```
cd /ds/ds-crazyflies
rm -rf build install log
./build.sh
source install/setup.bash
```

## ab jetzt in jedem Terminal
```
source install/setup.bash
```

## Drohne registrieren
```
ros2 service call /crazyflie_webots_gateway/add_crazyflie \
  crazyflie_webots_gateway_interfaces/srv/WebotsCrazyflie "{id: 0}"
```

## nodes starten
```
ros2 launch crazyflies framework.launch.py backend:=webots
```

## Hilfreiche Befehle

### vorhandene topics
```
ros2 topic list
```
### Nachrichtentyp des Topics rausfinden
```
ros2 topic info /cf0/cmd_position
```

Ausgabe:
```
Type: crazyflie_interfaces/msg/Position
Publisher count: 0
Subscription count: 1
```
### Struktur eines Nachrichtentyps ansehen

```
ros2 interface show crazyflie_interfaces/msg/Position
```
Ausgabe:
```
float32 x
float32 y
float32 z
float32 yaw
```
### Drohne abheben lassen
```
ros2 topic pub --once /cf0/cmd_position crazyflie_interfaces/msg/Position \
"{x: -2.0, y: 0.0, z: 2.5, yaw: 0.0}"
```


# Änderungen & Fixes für ds-crazyflies (Windows + Docker ohne GPU) (Rest kann man ignorieren)

Diese Datei fasst **alle Schritte, Fehlerbehebungen und Änderungen** zusammen, die notwendig waren, damit `ds-crazyflies` unter **Windows 11 + Docker Desktop (ohne NVIDIA GPU)** lauffähig wird.

---

## ✅ 1. Docker File und docker compose angepasst
```
services:
  ros-dev-windows:
    build:
      context: ../
      dockerfile: docker/Dockerfile
      args:
        ROS_DISTRO: humble
    image: ds-crazyflies:dev
    container_name: ds-crazyflies-dev
    ipc: host
    stdin_open: true
    tty: true
    environment:
      - DISPLAY=host.docker.internal:0.0
      - QT_X11_NO_MITSHM=1
      - USER=devuser
      - USERNAME=devuser
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    working_dir: /ds/ds-crazyflies
    command: bash
    privileged: true
```
```
# syntax=docker/dockerfile:1

# ROS distribution to use
ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop-full AS base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends ntpdate


# Install some basic packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip gdb dos2unix wget

RUN apt install mesa-utils -y 

# Install ros dependencies
RUN apt-get install -y ros-humble-tf-transformations ros-humble-ros2-control
# Install webots
RUN mkdir -p /etc/apt/keyrings
WORKDIR /etc/apt/keyrings
RUN wget -q https://cyberbotics.com/Cyberbotics.asc

RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
RUN apt update

RUN apt install webots -y

ENV WEBOTS_HOME=/usr/local/webots

RUN mkdir -p /ds && chmod 777 /ds
RUN useradd -ms /bin/bash devuser && \
    echo "devuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER devuser

RUN git config --global --add safe.directory /ds/ds-crazyflies

# Install ds-crazyflies
WORKDIR /ds
COPY --chown=devuser ./src /ds/ds-crazyflies/src
COPY --chown=devuser build.sh /ds/ds-crazyflies/

WORKDIR /ds/ds-crazyflies
RUN source /opt/ros/${ROS_DISTRO}/setup.bash &&\
    dos2unix build.sh &&\
    bash build.sh

# Install webotsworld
WORKDIR /ds
RUN git clone https://github.com/DynamicSwarms/crazywebotsworld.git
   
  
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /ds/ds-crazyflies/install/setup.bash" >> ~/.bashrc
```
---
## ✅ 2. framework.launch.py anpassen

### Änderungen:
- Hardware-Gateway komplett entfernt  
- Webots TCP/IP auf `host.docker.internal` gesetzt  
- "webots_use_tcp" auf True
- Crazyflie-Webots-Node aktiviert bzw hinzugefügt
- Wand und cf0 funktionieren jetzt zuverlässig  

```
import os

from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

from launch.actions import  DeclareLaunchArgument
from launch.conditions import LaunchConfigurationNotEquals
from launch_ros.actions import Node

WEBOTS_TCP_IP = "host.docker.internal"

def generate_launch_description():

    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="webots",
        description="Select used backend, choose 'webots', 'hardware' or 'both'.",
    )

    start_webots = LaunchConfigurationNotEquals("backend", "hardware")
    # This doesnt look too clean. In Jazzy we can use Substitions with Equals and Or

    webots_gateway = Node(
        condition=start_webots,
        package="crazyflie_webots_gateway",
        executable="gateway",
        name="crazyflie_webots_gateway",
        output="screen",
        parameters=[
            {
                "webots_port": 1234,
                "webots_use_tcp": True,
                "webots_tcp_ip": WEBOTS_TCP_IP,
            }
        ],
    )

    wand = Node(
        condition=start_webots,
        package="crazyflie_webots",
        executable="wand",
        name="Wand1",
        parameters=[
            {
                "id": 1,
                "webots_port": 1234,
                "webots_use_tcp": True,
                "webots_tcp_ip": WEBOTS_TCP_IP,
            }
        ],
        output="screen",
    )

    cf0 = Node(
        condition=start_webots,
        package="crazyflie_webots",
        executable="crazyflie",
        name="cf0",
        output="screen",
        parameters=[{
            "id": 0,
            "webots_port": 1234,
            "webots_use_tcp": True,
            "webots_tcp_ip": WEBOTS_TCP_IP,
        }],
    )

    position_visualization = Node(
        package="crazyflies",
        executable="position_visualization",
        name="position_visualization",
    )

    return LaunchDescription(
        [
            backend_arg,
            webots_gateway,
            wand,
            cf0,
            #hardware_gateway,
            position_visualization,
            #motion_caputre,
            #object_tracker,
        ]
    )

```
---
## ✅ 3. Docker Container starten

Container Tools installieren (Extenson in VS Code)

### In laufenden Container einsteigen
```powershell
docker exec -it ds-crazyflies-dev bash
```


---
## ✅ 4. Entfernen der Hardware-Abhängigkeiten

Da du keine Crazyflie-Hardware nutzt, mussten die Hardware-Pakete vollständig entfernt werden:

### Gelöscht:
im Container Hardware Pakete entferen
```
cd /ds/ds-crazyflies
rm -rf src/dependencies/crazyflie_hardware
```
Überprüfung ob noch doppelte Pakete vorhanden sind
```
find src -maxdepth 3 -type d -name crazyflie_interfaces
find src -maxdepth 3 -type d -name crazyflie_interfaces_python
```

Build Ordner löschen
```
rm -rf build install log
```

Workspace neu bauen
```
colcon build --symlink-install
```

Anleitung
```
cd /ds/ds-crazyflies
rm -rf build install log
./build.sh
source install/setup.bash
```


Dies entfernte doppelte ROS2-Pakete:
- `crazyflie_interfaces`
- `crazyflie_interfaces_python`

Damit wurde der Fehler behoben:

```
Duplicate package names not supported
```

---

## ✅ 5. Port 1234 war blockiert (Webots lief im Hintergrund) wurde glaube ich nicht gemacht

Fehler:
```
Ports are not available: exposing port 1234...
```

Lösung in PowerShell:
```
netstat -ano | findstr 1234
taskkill /PID <PID> /F
```

Erst danach konnte der Webots-ROS-Link funktionieren.

---

## ✅ 6. Fehlendes Paket `crazyflie-lib` manuell hinzugefügt (wurde glaube ich auch nicht gemacht)

Das Paket ist nicht öffentlich klonbar (GitHub verlangt Login).  
Wir haben eine **ZIP-basierte Kopie** in den Dependencies-Ordner gelegt:

```
src/dependencies/crazyflie-lib/
```

Damit wurden folgende Fehler gelöst:
- `Could not find package crtp_interfaces`
- `rqt_crazyflies` Build-Abbruch

---

## ✅ 7. Docker Fehler: Read-only File System


Beim Bauen wurde gemeldet:

```
[Errno 30] Read-only file system: '/ds/ds-crazyflies/install/...'
```

Fix:

docker compose angepasst

---

## ✅ 6. Erfolgreicher kompletter Build

Nach allen Fixes lief der Build durch:

```
11 packages finished successfully
```

Damit starteten:
- `/cf0` – Crazyflie Webots Node  
- `/Wand1` – Tastatursteuerung  
- `/crazyflie_webots_gateway` – Kommunikationslink  
- `/position_visualization`  

---

## ✅ 7. Webots ↔ Docker Kommunikation funktioniert

Webots Log:

```
INFO: 'Wand1' extern controller: connected.
```

ROS2 Log:

```
[cf0]: CrazyflieWebots node initialized.
```

Damit ist die Simulation **voll einsatzbereit**.

---

## 🚁 8. Drohne fliegen ohne rqt

rqt funktioniert unter Docker + Windows nicht zuverlässig wegen OpenGL.  
Aber es ist *nicht notwendig*, um die Simulation zu steuern.

### Wand-Steuerung (Tastatur):
- Pfeiltasten → vor/zurück/links/rechts  
- W / S → hoch / runter  
- Q / E → drehen  
- A / D → Modus wechseln  
- 1 / 2 / … → Controller wählen  

---

# 🎉 Ergebnis

Dein System funktioniert jetzt vollständig:

- ✔ Webots GUI läuft unter Windows  
- ✔ ROS2 Nodes laufen im Docker-Container  
- ✔ Wand-Steuerung arbeitet korrekt  
- ✔ cf0 fliegt in Webots  
- ✔ Alle Build-Fehler wurden behoben  
- ✔ Keine Hardware-Abhängigkeiten  
- ✔ rqt wird nicht benötigt  

---

