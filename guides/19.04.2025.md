# URDF + Moveit

## Содержание
- [0. Введение](#0-введение)
- [1. Создание пакета](#1-создание-пакета)
- [2. Создание URDF](#2-создание-urdf)
  - [2.1. Макрос robot_link_macros.xacro](#21-макрос-robot_link_macrosxacro)
  - [2.2. Макрос test_robot_macros.urdf.xacro](#22-макрос-test_robotmacrosurdfxacro)
  - [2.3. Файл test_robot.urdf.xacro](#23-файл-test_roboturdfxacro)
  - [2.4 Сборка пакета](#24-сборка-пакета)
  - [2.5. Проверка URDF файла](#25-проверка-urdf-файла)
  - [2.6 Визуализация URDF](#26-визуализация-urdf)
  - [2.7 Создаем robot state publisher](#27-создаем-robot-state-publisher)
  - [2.8 Визуализация в RViz](#28-визуализация-в-rviz)
- [3. Создание MoveIt конфигурации](#3-создание-moveit-конфигурации)
- [4. Планирование траектории](#4-планирование-траектории)
- [5. Домашнее задание](#5-домашнее-задание)


### 0. Введение
В этом руководстве мы создадим URDF файл для вашего робота и настроим MoveIt для управления им.  

**MoveIt** - это фреймворк для планирования траекторий и управления роботами в ROS. Она позволяет создавать сложные движения робота, используя различные алгоритмы планирования и управления.  
**URDF** (Unified Robot Description Format) - это формат файла, который используется для описания роботов в ROS. Он позволяет описывать геометрию, кинематику и динамику робота. URDF файлы могут быть использованы для визуализации робота в RViz, а также для планирования траекторий с помощью MoveIt.  
**Xacro** - это расширение URDF, которое позволяет использовать макросы и параметры для создания URDF файлов. Это упрощает создание и редактирование URDF файлов, особенно для сложных роботов. Xacro файлы имеют расширение .xacro и могут быть преобразованы в URDF файлы с помощью инструмента xacro.  

После прохождения этого руководства у вас будет рабочая конфигурация MoveIt для вашего робота. Также вы научитесь использовать Python-API MoveIt для планирования траекторий и управления роботом.
![Preview](/guides/media/19.04.2025_preview.png)

**Note:** Преполагается что вы уже умеете пользоваться инструментами ROS2 Humble и создали workspace
### 1. Создание пакета
С помощью команды ros2 pkg create создайте новый пакет для вашего робота. Например, назовем его test_robot_description.
```bash
cd ~/ros_ws/src
ros2 pkg create --build-type ament_cmake  test_robot_description
```
Эта команда создаст пакет с именем test_robot_description в директории ~/ros_ws/src. Пакет будет использовать CMake в качестве системы сборки.
Создайте директорию для хранения файлов URDF и MoveIt конфигурации
```bash
mkdir -p test_robot_description/urdf/include
mkdir -p test_robot_description/launch
cd test_robot_description/urdf
```
В файл *test_robot_description/CMakeLists.txt* добавьте следующие зависимости:
```cmake
install(DIRECTORY 
  urdf 
  launch 
  DESTINATION share/${PROJECT_NAME}/)
```
Также добавьте зависимости в файл *package.xml*:
```xml
<build_depend>xacro</build_depend>
<exec_depend>xacro</exec_depend>
```

### 2. Создание URDF
Для создания URDF мы будем использовать инструмент xacro, он позволяет создавать URDF файлы с использованием макросов и параметров. Это упрощает создание и редактирование URDF файлов, особенно для сложных роботов.  
Создадим файл URDF для вашего робота. Например, назовем его test_robot.urdf.xacro. Вы можете использовать любой текстовый редактор для создания этого файла.
```bash
touch test_robot.urdf.xacro
touch test_robot_macros.urdf.xacro
touch include/robot_link_macros.xacro
```
**Note**: если вы не можете редактировать файл извне контейнера, выполните команду `sudo chown -R $USER:$USER ./` вне контейнера, чтобы изменить владельца папки на текущего пользователя.

#### 2.1. Макрос robot_link_macros.xacro
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- Xacro макрос для создания прямоугольных звеньев -->
    <xacro:macro name="robot_link" params="name length width height color">
        <link name="${name}">
            <visual>
                <origin xyz="0 0 ${length/2}" rpy="0 1.5708 0" />
                <geometry>
                    <box size="${length} ${width} ${height}" />
                </geometry>
                <material name="${color}" />
            </visual>
            <collision>
                <origin xyz="0 0 ${length/2}" rpy="0 1.5708 0" />
                <geometry>
                    <box size="${length} ${width} ${height}" />
                </geometry>
            </collision>
            <inertial>
                <origin xyz="0 0 ${length/2}" rpy="0 1.5708 0" />
                <mass value="1.0" />
                <inertia
                    ixx="0.01" ixy="0.0" ixz="0.0"
                    iyy="0.01" iyz="0.0"
                    izz="0.01" />
            </inertial>
        </link>
    </xacro:macro>

    <!-- Xacro макрос для создания вращающегося соединения -->
    <xacro:macro name="revolute_joint"
        params="name parent child axis origin_xyz origin_rpy limit_lower limit_upper">
        <joint name="${name}" type="revolute">
            <parent link="${parent}" />
            <child link="${child}" />
            <origin xyz="${origin_xyz}" rpy="${origin_rpy}" />
            <axis xyz="${axis}" />
            <limit lower="${limit_lower}" upper="${limit_upper}" effort="100" velocity="1.0" />
        </joint>
    </xacro:macro>
</robot>
```
#### 2.2. Макрос test_robot_macros.urdf.xacro
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- Подключаем макросы звеньев и соединений -->
    <xacro:include filename="$(find test_robot_description)/urdf/include/robot_link_macros.xacro" />
    
    <xacro:macro name="six_axis_robot" params="prefix">
        <material name="blue">
            <color rgba="0 0 1 1" />
        </material>
        <!-- Описываем звенья (прямоугольные блоки) -->
        <xacro:robot_link name="${prefix}link_1" length="0.3" width="0.05" height="0.05" color="blue" />
        <xacro:robot_link name="${prefix}link_2" length="0.3" width="0.045" height="0.045" color="blue" />
        <xacro:robot_link name="${prefix}link_3" length="0.25" width="0.045" height="0.045" color="blue" />
        <xacro:robot_link name="${prefix}link_4" length="0.2" width="0.04" height="0.04" color="blue" />
        <xacro:robot_link name="${prefix}link_5" length="0.15" width="0.035" height="0.035" color="blue" />
        <xacro:robot_link name="${prefix}link_6" length="0.1" width="0.03" height="0.03" color="blue" />
        <link name="${prefix}end_effector">
            <visual>
                <origin xyz="0 0 0" rpy="0 0 0" />
                <geometry>
                    <box size="0.05 0.05 0.05" />
                </geometry>
                <material name="blue" />
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0 0 0" />
                <geometry>
                    <box size="0.05 0.05 0.05" />
                </geometry>
            </collision>
        </link>
        <!-- Описываем подвижные соединения -->
        <xacro:revolute_joint name="${prefix}joint_1" 
            parent="base_link" child="${prefix}link_1"
            axis="0 0 1" origin_xyz="0 0 0.05" origin_rpy="0 0 0"
            limit_lower="-3.14" limit_upper="3.14" />

        <xacro:revolute_joint name="${prefix}joint_2" 
            parent="${prefix}link_1"
            child="${prefix}link_2"
            axis="0 1 0" origin_xyz="0 0 0.3" origin_rpy="0 0 0"
            limit_lower="-3.14" limit_upper="3.14" />

        <xacro:revolute_joint name="${prefix}joint_3"
            parent="${prefix}link_2"
            child="${prefix}link_3"
            axis="0 1 0" origin_xyz="0 0 0.3" origin_rpy="0 0 0"
            limit_lower="-3.14" limit_upper="3.14" />

        <xacro:revolute_joint name="${prefix}joint_4"
            parent="${prefix}link_3"
            child="${prefix}link_4"
            axis="0 1 0" origin_xyz="0 0 0.25" origin_rpy="0 0 0"
            limit_lower="-3.14" limit_upper="3.14" />

        <xacro:revolute_joint name="${prefix}joint_5"
            parent="${prefix}link_4"
            child="${prefix}link_5"
            axis="1 0 0" origin_xyz="0 0 0.2" origin_rpy="0 0 0"
            limit_lower="-3.14" limit_upper="3.14" />

        <xacro:revolute_joint name="${prefix}joint_6"
            parent="${prefix}link_5"
            child="${prefix}link_6"
            axis="0 0 1" origin_xyz="0 0 0.15" origin_rpy="0 0 0"
            limit_lower="-3.14" limit_upper="3.14" />

        <!-- Конечный элемент (фиксирован) -->
        <joint name="${prefix}ee_fixed" type="fixed">
            <parent link="${prefix}link_6" />
            <child link="${prefix}end_effector" />
            <origin xyz="0 0 0.1" rpy="0 0 0" />
        </joint>
    </xacro:macro>
</robot>
```
#### 2.3. Файл test_robot.urdf.xacro
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="six_axis_robot">
    <xacro:include filename="$(find test_robot_description)/urdf/test_robot_macros.urdf.xacro" />
    <!-- Создаем базовое звено, точку крепления -->
    <link name="base_link" />
    <xacro:six_axis_robot
        prefix="r1_" />
</robot>
```
#### 2.4 Сборка пакета
```bash
cd ~/ros_ws
colcon build
source install/setup.zsh
```
#### 2.4. Проверка URDF файла
Для проверки URDF файла мы будем использовать инструмент *xacro*
```bash
xacro ~/ros_ws/install/test_robot_description/share/test_robot_description/urdf/test_robot.urdf.xacro
```
В выводе вы должны увидеть URDF файл вашего робота. Если вы видите ошибки, проверьте ваш URDF файл на наличие ошибок.
#### 2.5 Визуализация URDF
Для визуализации URDF файла мы будем использовать инструмент *urdf-viz*. Он уже установлен в docker-образе.
```bash
urdf-viz ~/ros_ws/install/test_robot_description/share/test_robot_description/urdf/test_robot.urdf.xacro
```
#### 2.6 Создаем robot state publisher
Для того чтобы опубликовать URDF модель робота и положение суставов, мы будем использовать пакет robot_state_publisher. В нем уже есть узел, который публикует трансформации между звеньями робота на основе URDF файла. Чтобы его использовать, создайте файл robot_state_publisher.launch.py в директории launch вашего пакета test_robot_description.
```bash
mkdir -p ~/ros_ws/src/test_robot_description/launch
touch ~/ros_ws/src/test_robot_description/launch/robot_state_publisher.launch.py
```
В файл robot_state_publisher.launch.py добавьте следующий код:
```python
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
    OpaqueFunction,)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (LaunchConfiguration,
    PathJoinSubstitution)
import xacro


def launch_setup(context, *args, **kwargs):
    descritpion_package_name = LaunchConfiguration("package_name")
    descritpion_file_name = LaunchConfiguration("description_file")
    robot_description_path = PathJoinSubstitution(
        [FindPackageShare(descritpion_package_name), "urdf", descritpion_file_name]
    ).perform(context)
    robot_description = xacro.process_file(robot_description_path,
                                           mappings={}).toprettyxml(indent='  ')
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {"robot_description": robot_description}],
            output='screen'
        )
    rqt_joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )
    nodes = [
        robot_state_publisher_node,
        rqt_joint_state_publisher_node
    ]
    return nodes


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'package_name',
            default_value='test_robot_description',
            description='Package name of the robot description'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='test_robot.urdf.xacro',
            description='URDF file name'
        )
    )
    return LaunchDescription(declared_arguments
                            + [OpaqueFunction(function=launch_setup)])

```
Заново соберите пакет и запустите launch файл.
```bash
cd ~/ros_ws
colcon build && source install/setup.zsh
ros2 launch test_robot_description robot_state_publisher.launch.py
```
#### 2.7 Визуализация в RViz
Для визуализации модели робота в RViz, откройте RViz 2 и в Global Options выберите Fixed Frame: `base_link`. Затем в разделе "Add" выберите **RobotModel**. В поле "Description Topic" укажите `/robot_description`. Вы должны увидеть модель вашего робота.  
Launch файл также запускает GUI для управления суставами робота. Вы можете управлять суставами робота с помощью GUI и видеть изменения в RViz.

Теперь добавьте **TF** в RViz, чтобы увидеть трансформации между звеньями робота. Для этого в разделе "Add" выберите **TF**.

### 3. Создание MoveIt конфигурации
Для создания MoveIt конфигурации мы будем использовать инструмент MoveIt Setup Assistant. Он позволяет создавать конфигурацию для вашего робота с помощью графического интерфейса.
```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```
После запуска инструмента, выберите "Create New MoveIt Configuration Package" и укажите путь к вашему URDF файлу (*test_robot.urdf.xacro*). Затем выберите "Load Files".  
На экране *Self Collision* выберите "Generate Collision Matrix" и переходите в "Planning Groups".

В разделе "Planning Groups" выберите "Add Group" и создайте группу для вашего робота. Например, назовем ее *arm*. Kinemactic solver: *kdl_kinematics_plugin/KDLKinematicsPlugin*. Затем "Add Joints" и выберите все суставы вашего робота кроме "ee_fixed". В "Chain" укажите:
- Base Link: base_link
- Tip Link: end_effector

В "ros2_control" выберите "position" и "velocity" для command и state interfaces и нажмите "Add Interfaces".  
В "MoveIt Controllers" нажмите "Auto Add Follow Joint Trajectory Controllers..."
Для создания пакета измените *Author Information* на свои данные и перейдите в "Configuration Files", затем выберите путь для сохранения пакета "/root/ros_ws/src/test_robot_moveit_config" и нажмите "Generate Package".
После этого необходимо собрать пакет и запустить MoveIt.
```bash
cd ~/ros_ws
colcon build
source install/setup.zsh
ros2 launch test_robot_moveit_config demo.launch.py
```
Вы получите ошибку:
```bash
what():  parameter 'robot_description_planning.joint_limits.r1_joint_1.max_velocity' has invalid type: expected [double] got [integer]
```
Это происходит из-за того, что в файле *test_robot_moveit_config/config/joint_limits.yaml* указаны значения max_velocity и max_acceleration как целые числа. Измените их на числа с плавающей точкой. Также необходимо указать максимальное ускорение для привода.
В итоге должно получиться что-то вроде:
```yaml
joint_limits:
  r1_joint_1:
    has_velocity_limits: true
    max_velocity: 1. # max velocity in rad/s
    has_acceleration_limits: true
    max_acceleration: 1. # max acceleration in rad/s^2
...
```
Затем еще раз запустите RViz и MoveIt.
```bash
ros2 launch test_robot_moveit_config demo.launch.py
```
Теперь вы можете управлять суставами робота с помощью GUI и видеть изменения в RViz. Также можете использовать MoveIt для планирования траекторий и управления роботом.

### 4. Планирование траектории
Для планирования траектории мы будем использовать Python-биндинги MoveIt. Они реализованы в пакете moveit_py. Он уже установлен в docker-образе.
В пакете [test_robot_moveit_py](/ros_ws/src/test_robot_moveit_py) уже есть пример, который показывает, как использовать Python-API MoveIt для планирования траекторий и управления роботом. Запустите его командой:
```bash
ros2 run test_robot_moveit_py example
```
Если у вас запущен launch, то робот придет в точку `[-0.3, 0.2, 1.0]`:
```bash
ros2 launch test_robot_moveit_config demo.launch.py
```

### 5. Домашнее задание
1. Пройдите туториал и пришлите скринкаст RViz как работает скрипт из пункта 4.
2. Модифицируйте python-скрипт для выполнения двух траекторий на выбор, например: движение по окружности, движение по квадрату.
3. Добавьте линейную ось в основание робота и спланируйте траекторию с ней.
4. **Advanced** Реализуйте управление в joint-space, без использования метода `robot_planning.plan()`.
5. **Advanced** Найдите в интернете description-пакет готового робота и реализуйте управление им из python.
