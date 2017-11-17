## Remote deployment of AVR binaries

### Setup
```
cd ~/catkin_ws/src/kmm/embedded
chmod +x avrdeploy.bash

sudo ln -s /home/$USER/catkin_ws/src/kmm/embedded/avrdeploy.bash /usr/local/bin/avrdeploy
```

### Usage
```
cd ~/catkin_ws/src/kmm/embedded

cd motor
# or
cd sensor

make deploy
```