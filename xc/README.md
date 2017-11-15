### Installation of docker and other dependencies.
```
# install docker
sudo apt-get update

sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common
    
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"

sudo apt-get update
sudo apt-get install docker-ce

# install qemu 
sudo apt-get install qemu qemu-user-static debootstrap
```

### Building docker image
```
cd ~/catkin_ws/src/kmm/xc/
sudo docker build -t kmm_xc .
```

### Setting up environment
```
mkdir ~/catkin_ws_arm

echo "export CATKIN_WS=\"/home/$USER/catkin_ws/\"" >> ~/.xcenv
echo "export CATKIN_WS_ARM=\"/home/$USER/catkin_ws_arm/\"" >> ~/.xcenv

sudo ln -s /home/$USER/catkin_ws/src/kmm/xc/xc.bash /usr/local/bin/xc
```

### Compiling for ARM
Start compilation container:
```
sudo xc start
```

Write some code and compile:
```
sudo xc make
```

Push the compiled binaries to the robot (requires $ROS_MASTER_URI to be set):
```
sudo xc push
```


And when you are done for the day:
```
sudo xc stop
```