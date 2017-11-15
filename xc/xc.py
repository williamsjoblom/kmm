#!/usr/bin/env python
import sys, os
import docker

from subprocess import check_output, CalledProcessError

def write_id(id):
    """
    Write given container id to ./.id
    """
    with open('.id', 'w') as f:
        f.truncate()
        f.write(id)

        
def read_id():
    """
    Read running container id from ./.id
    """
    with open('.id', 'r') as f:
        return f.readline()

    
def running_container(client):
    """
    Return running compilation container.
    """
    id = read_id()
    return client.containers.get(id)
    

def build():
    """
    Build docker container
    """
    print("Building container, this may take a while!")
    c = ['docker', 'build', '-t', 'kmm_xc', '.']
    try:
        result = check_output(c).strip()
        success = True 
    except CalledProcessError as e:
        result = e.output.decode()
        success = False
        
    if not success:
        print("Failed to build container:")
        print(result)
        
    
def start(client):
    """
    Start docker container.
    """
    if len(sys.argv) != 3 or not os.path.isdir(sys.argv[2]):
        print('Syntax: ' + sys.argv[0] + ' start <catkin_ws path>')
        sys.exit(1)

    path = os.path.abspath(sys.argv[2])
    v = {path: {'bind': '/root/catkin_ws', 'mode': 'rw'}}
    
    container = client.containers.run('kmm_xc', '/bin/bash', detach=True,
                                      volumes=v, tty=True, stdin_open=True)
    write_id(container.id)
    return container


def stop(client):
    """
    Stop docker container.
    """
    running_container(client).stop()

    
def compile(client):
    """
    Run compilation.
    """
    cmd = 'bash -c "source /opt/ros/kinetic/setup.bash && cd /root/catkin_ws && catkin_make"'
    api_client = docker.APIClient()
    
    exec_id = api_client.exec_create(read_id(), cmd)
    stream = api_client.exec_start(exec_id, stream=True)

    for s in stream:
        print(s.strip())
            
        
def bad_directive():
    """
    Print help
    """
    print("Expected directive:")
    print("build   -- Build compilation container")
    print("start   -- Start compilation container")
    print("stop    -- Stop compilation container")
    print("compile -- Compile")
    sys.exit(1)
    
if __name__ == "__main__":
    if len(sys.argv) <= 1:
        bad_directive()

    client = docker.from_env()
        
    directive = sys.argv[1]
    if directive == "build":
        build()
    elif directive == "start":
        start(client)
    elif directive == "stop":
        stop(client)
    elif directive == "compile":
        compile(client)
    else:
        bad_directive()



