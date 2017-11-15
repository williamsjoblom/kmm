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
sudo docker build -t kmm_xc .
```

### Compiling for ARM
Start compilation container:
```
sudo ./xc.py start ~/catkin_ws
```

Write some code and compile:
```
sudo ./xc.py compile
```

And when you are done for the day:
```
sudo ./xc.py stop
```