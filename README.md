# unitree_h1_teleoperation_ws
В данном репозитории лежат ROS2-пакеты, которые позволяют реализовать телеуправление роботом Unitree H1 с помощью Устройства Копирующего Типа (УКТ) от НПО "Андроидная техника".

## 📦 Содержание Репозитория
* **`completed_scripts_teleoperation`**: Содержит launch-файлы, которые запускают набор нод из данного репозитория.
* **`converter_from_fedor_into_rad_package`**: Содержит ноду, которая конвертирует значения, полученные с УКТ, в радианы.
*   **`converter_from_fedor_to_h1_package`**: Содержит ноду, которая конвертирует значения, полученные с УКТ, в формат для управления Unitree H1 (low_level_control).
*   **`extractor_package`**: Содержит ноду, которая позволяет извлечь координату определённого звена с Unitree H1 и УКТ одновременно и републикует их в отдельные ROS2-топики, что удобно для снятия показателей качества телеуправления.
*   **`repeater_package`**: Содержит ноду, которая ретранслирует данные с UDP-сокета в ROS2-топик 
*   ==**`docs/`**: Дополнительная документация (если есть)==.
*   **`README.md`**: Этот файл.
*   **`save.bash`**: Скрипт для быстрой выгрузки на github

## 🚀 Быстрый Старт
Пошаговая инструкция для **быстрого** запуска основной функциональности. Предполагаем, что ROS2 уже установлен.

1.  **Клонировать репозиторий** в `src` вашего workspace:
```bash
mkdir -p unitree_h1_teleoperation_ws/src
cd unitree_h1_teleoperation_ws/src
git clone https://github.com/cyberbanana777/unitree_h1_teleoperation_ws.git .
```
2.  2. **Установить проприетарные зависимости** (по ссылкам инструкции по установке от производителя): 
-   [unitree_sdk2py](https://github.com/unitreerobotics/unitree_sdk2_python) При установке данного пакета нужно выполнить инструкции из `Installing from source` и установить пакет из исходников, т.к. там программа новее, чем представлена в pip. Для корректной работы перед установкой нужно подправить 1 файл в данном пакете. В нём нужно убрать в первой строчке в конце импорт b2 и удалить 9 строчку вовсе. После этого можно устанавливать пакет в систему как описано в README на github. Команда, открывающая заветный файл:
```bash 
~/unitree_sdk2_python/unitree_sdk2py/__init__.py
```
-   [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) Здесь мы выполняем всё, что написано до `Connect to Unitree robot`. После всех действий нужно выполнить команду и перезапустить терминал:
```bash
echo "source ~/unitree_ros2/cyclonedds_ws/install/setup.bash" >> ~/.bashrc
```
3. **Установить зависимости**:
```bash
cd unitree_h1_teleoperation_ws
rosdep install --from-paths src --ignore-src -y  # Основные ROS зависимости
```
4.  **Собрать workspace:**
```bash
colcon build
source install/setup.bash  # Или setup.zsh - в зависимости от вашего интерпретатора командной строки
```
5.  **Запустить пример / основной функционал:**
```bash
ros2 launch completed_scripts_teleoperation teleoperation_launch.py
```

## ⚙️ Предварительные Требования

Детализируйте *все*, что нужно *до* шагов "Быстрого Старта":
*   **Поддерживаемые версии ROS2:** Foxy
*   **Поддерживаемые платформы:** Ubuntu 20.04
*   **Ключевые ROS2 пакеты:** `rclpy`, `std_msgs`, geometry_msgs`, `unitree_go`

## 🧪 Использование

Как пользоваться пакетами *после* установки и сборки.
### **Запуск узлов:**
#### converter_angles_fedor_into_rad_node
```bash 
ros2 run converter_angles_fedor_into_rad_package converter_angles_fedor_into_rad_node
```
#### converter_from_fedor_to_h1_package
```bash
ros2 run converter_from_fedor_to_h1_package converter_from_fedor_to_h1_node
```
#### extractor_package
##### Без параметра
```bash
ros2 run extractor_package extractor_node
```
##### С параметром
```bash
ros2 run extractor_package extractor_node --ros-args -p H1_joint_num:=15 # С параметром значения типа int
```
#### repeater_package
```bash
ros2 run repeater_package repeater_node 
```
### **Запуск Launch файлов:**
#### Запуск основного функционала
```bash
ros2 launch completed_scripts_teleoperation teleoperation_launch.py
```
#### Запуск основного функционала с метриками 
```bash
ros2 launch completed_scripts_teleoperation teleoperation_with_metrics_launch.py
```


## 📡 Интерфейс (Топики, Сервисы, Действия, Параметры)

Детальная спецификация API пакетов. Обычно таблицы.

### **Пакет 1: converter_angles_fedor_into_rad_node**
#### **Узел: `converter_angles_fedor_into_rad_node `**
- **Публикуемые топики:**

| Тип услуги | Топик              | Тип сообщения         | Описание                            |
| :--------- | :----------------- | :-------------------- | :---------------------------------- |
| Публикация | `/Fedor_data_rad`  | `std_msgs/msg/String` | Значения координат с УКТ в радианах |
| Подписка   | `/Fedor_bare_data` | `std_msgs/msg/String` | Исходные значения координат с УКТ   |

- **Параметры:**

| Параметр        | Тип (знач. по умол.) | Описание                                                               |
| :-------------- | :------------------- | :--------------------------------------------------------------------- |
| `scaling_param` | `float (0.1)`        | Параметр Scale, который задаётся в unity-прогамме от производителя УКТ |

### **Пакет 2: converter_angles_fedor_into_rad_node**
#### **Узел: `converter_from_fedor_to_h1_node `**
- **Публикуемые топики:**

| Тип услуги | Топик                   | Тип сообщения         | Описание                                                    |
| :--------- | :---------------------- | :-------------------- | :---------------------------------------------------------- |
| Публикация | `/positions_to_unitree` | `std_msgs/msg/String` | Целевые значения координатах в системе координат Unitree H1 |
| Подписка   | `/Fedor_bare_data`      | `std_msgs/msg/String` | Исходные значения координат с УКТ в JSON-формате            |

### **Пакет 3: extractor_package**
#### **Узел: `extractor_node `**
- **Публикуемые топики:**

| Тип услуги   | Топик                                         | Тип сообщения                | Описание                                                           |
| :----------- | :-------------------------------------------- | :--------------------------- | :----------------------------------------------------------------- |
| Публикация   | `/plotjuggler/joint_{H1_joint_num}/h1`        | `std_msgs/msg/Float32`       | Значение координаты конкретного звена с УКТ  в радианах            |
| Публикация   | `/plotjuggler/joint_{H1_joint_num}/fedor`<br> | `std_msgs/msg/Float32`<br>   | Значение координаты конкретного звена с Unitree H1  в радианах<br> |
| Подписка     | `/Fedor_data_rad`                             | `std_msgs/msg/String         | Значения координат с УКТ в радианах                                |
| Подписка<br> | `/inspire/state`                              | `unitree_go/msg/MotorStates` | Значения координат пальцев `Inspire Hands`                         |
| Подписка<br> | `/lowstate`                                   | `unitree_go/msg/LowState`    | Значения координат робота `Unitree H1`                             |

- **Параметры:**

| Параметр       | Тип (знач. по умол.) | Описание                                       |
| :------------- | :------------------- | :--------------------------------------------- |
| `H1_joint_num` | `int (16)`           | Номер звена, точность которого хотим отследить |

### **Пакет 4: repeater_package**
#### **Узел: `repeater_node `**
- **Публикуемые топики:**

| Тип услуги | Топик              | Тип сообщения         | Описание                                         |
| :--------- | :----------------- | :-------------------- | :----------------------------------------------- |
| Публикация | `/Fedor_bare_data` | `std_msgs/msg/String` | Исходные значения координат с УКТ в JSON-формате |


## 🗺️ Архитектура 
Диаграмма взаимодействия представлена тут.
## Потенциальные улучшения
### converter_from_fedor_to_h1_package
- Почистить код
- PEP8
### converter_from_fedor_into_rad_package и extractor_package (для Алисы)
- Проблема в переводе из сопоставления одних joints с другими, можно только 1 раз переводить в joints unitree h1 и упростить оба кода

## Предложения и корректировки
Если Вы нашли, ошибку, неточность, у Вас есть предложения по улучшению или вопросы, то напишите в телеграмм [сюда](https://t.me/Alex_19846) (Александр) или [сюда](https://t.me/Kika_01) (Алиса).