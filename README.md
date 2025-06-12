# Порядок запуска 

## Запуск мира и модели turtlebot3:
```bash
roslaunch turtlebot3_lidar_tracker wall_world.launch
```

## Запуск фильтра для scan:
```bash
rosrun turtlebot3_lidar_tracker scan_filter.py
```

## Запуск трекера движения:
```bash
roslaunch turtlebot3_lidar_tracker wall_motion.launch
```

## Запуск teleopt key для управления:
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
