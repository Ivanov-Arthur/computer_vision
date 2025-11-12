# Aruco detector

## Простой детектор меток аруко. Ничего не делает пока что, просто распознает метку аруко, обводит её границы зеленым цветом, распознает айди и выводит на экран айди. (вебкамера)

сборка запуск

```bash
colcon build --packages-select aruco_detector
source install/setup.bash
ros2 run aruco_detector aruco_detector
```
