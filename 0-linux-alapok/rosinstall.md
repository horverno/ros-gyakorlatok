## ROS telepítés

Ha a `roscore` parancsra command not found a válasz, telepítsük az ROS-t.

A leírás következő rendszereket feltételezi:
- Ubuntu MATE 16.04 vagy Ubuntu 16.04
- ROS Kinetic (*ez fog települni*)

Adjuk ki a következő parancsokat terminalból:

Először állítsuk be gépet, hogy a `packages.ros.org`-ról fogadjon szoftvereket:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Állítsuk be a kulcsokat:

```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

```
sudo apt-get update
```

A következő parancs lesz a leghosszabb lefutású:
```
sudo apt-get install ros-kinetic-desktop-full
```

```
apt-cache search ros-kinetic
```

```
sudo rosdep init
```

```
rosdep update
```

Kényelmes, ha a ROS környezeti változók minden új terminál (shell) nyitáskor automatikusan bash sessionhoz adódnak:

```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
```

```
source ~/.bashrc
```

```
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## Részletesebb leírás

- http://wiki.ros.org/kinetic/Installation/Ubuntu

## Kezdőoldal

GitHub Pages kezdőoldal: 
[horverno.github.io/ros-gyakorlatok](https://horverno.github.io/ros-gyakorlatok/)