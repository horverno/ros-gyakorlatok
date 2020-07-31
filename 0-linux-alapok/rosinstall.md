## ROS telepítés

Ha a `roscore` parancsra command not found a válasz, telepítsük az ROS-t.

A leírás következő rendszereket feltételezi:
- Ubuntu MATE 18.04 vagy Ubuntu 18.04
- ROS Melodic (*ez fog települni*)

Adjuk ki a következő parancsokat terminalból:

Először állítsuk be gépet, hogy a `packages.ros.org`-ról fogadjon szoftvereket:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Állítsuk be a kulcsokat:

```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

```
sudo apt-get update
```

A következő parancs lesz a leghosszabb lefutású:
```
sudo apt install ros-melodic-desktop-full
```

```
apt-cache search ros-melodic
```

```
sudo rosdep init
```

```
rosdep update
```

Kényelmes, ha a ROS környezeti változók minden új terminál (shell) nyitáskor automatikusan bash sessionhoz adódnak:

```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```

```
source ~/.bashrc
```

```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
```

## Részletesebb leírás

- http://wiki.ros.org/melodic/Installation/Ubuntu

## Kezdőoldal

GitHub Pages kezdőoldal: 
[horverno.github.io/ros-gyakorlatok](https://horverno.github.io/ros-gyakorlatok/)
