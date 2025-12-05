# ROS2
## launch anpassen
```
docker cp "C:\git\2_Semester_Master\ds-crazyflies\src\crazyflies\launch\framework.launch.py" ds-crazyflies-dev:/ds/ds-crazyflies/src/crazyflies/launch/framework.launch.py
```

## in Docker geänderte launch aktivieren
```
source install/setup.bash
ros2 launch crazyflies framework.launch.py backend:=webots
```
## Drohne registrieren
```
ros2 service call /crazyflie_webots_gateway/add_crazyflie \
  crazyflie_webots_gateway_interfaces/srv/WebotsCrazyflie "{id: 0}"
```
## vorhandene topics
```
ros2 topic list
```
## Nachrichtentyp des Topics rausfinden
```
ros2 topic info /cf0/cmd_position
```

Ausgabe:
```
Type: crazyflie_interfaces/msg/Position
Publisher count: 0
Subscription count: 1
```
## Struktur eines Nachrichtentyps ansehen

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
## Drohne abheben lassen
```
ros2 topic pub --once /cf0/cmd_position crazyflie_interfaces/msg/Position \
"{x: 2.0, y: 0.0, z: 0.5, yaw: 0.0}"
```
## manuelles testen der Befehle
```
ros2 topic pub /crazyflie/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.3}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
