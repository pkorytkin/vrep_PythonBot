# vrep_PythonBot

**Внутри репозитория:**

**1.** Пакет ROS с мозгами для движения робота. Является форком моего репозитория https://github.com/pkorytkin/turtlesim_polygonal_brain

**2.** Сцена Vrep с модифицированным мной стандартного BubbleBot с прикреплёнными написанными скриптами на Python для взаимодействия с ROS.

**Установка и запуск:**

1. Колнируем репозиторий в ~catkin_ws/src

2. Открываем в VREP https://www.coppeliarobotics.com/downloads -> vrep_PythonBot/PythonBotVREP.ttt

3. Запускаем roscore

4. Запускаем сцену робота в VREP нажав кнопку Play.

5. Запускаем пакет:

python vrep_PythonBot/src/brain.py 

или 

rosrun vrep_PythonBot launch.launch

6. Profit!

**Функционал PythonBot:**

**1.** Публикация в ROS изображения с камеры в топик /image - sensor_msgs/Image

**2.** Публикация в ROS показания сенсора о наличии препятствия /sensor - std_msgs/Bool

**3.** Публикация в ROS показания лазерного дальномера (расстояние до объекта из сенсора) /laser - std_msgs/Float32

**4.** Публикация в ROS показания позиции 2D робота /pose - geometry_msgs/Pose2D

**5.** Публикация в ROS времени симуляции /simTime - std_msgs/Float32

**6.** Подписчик в ROS для управления роботом /cmd_vel - geometry_msgs/Twist

**7.** Подписчик в ROS из VREP для проверки получаемого изображения из /image и отображения в VREP /passiveSensor (вдруг изображение сломалось в пути. Бывает баг, когда изображение почему-то перевёрнуто).
