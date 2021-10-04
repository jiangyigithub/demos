# Enviroment Deployment

# open terminal
- terminal
Ctrl+Alt+T
Ctrl+Shift+T

- vscode
Ctrl+ `
Ctrl+Shift+5


# linux command
  378  rm -rf install log build

  382  cd difusion/
  383  ls
  385  mv CMakeLists.txt CMakeLists.txt.bkup
  386  mv package.xml package.xml.bkup
  393  gedit CMakeLists.txt.bkup 
  395  gedit package.xml.bkup 

  400  cd src
  401  ls
  402  cp -rf difusion difusion_test
  403  cd difusion_test
  404  ls
  405  mv CMakeLists.txt.bkup CMakeLists.txt
  406  gedit CMakeLists.txt 
  407  mv package.xml.bkup package.xml
  408  gedit package.xml

# virual machine
- shared file
  https://www.pragmaticlinux.com/2021/02/how-to-mount-a-shared-folder-in-virtualbox/#:~:text=Mount%20a%20shared%20folder%20manually.%20After%20disabling%20the,following%20command%20to%20manually%20mount%20the%20shared%20folder%3A

- share copy from windows
  Specifying a specific mount point: https://www.pragmaticlinux.com/2021/02/how-to-mount-a-shared-folder-in-virtualbox/#:~:text=Mount%20a%20shared%20folder%20manually.%20After%20disabling%20the,following%20command%20to%20manually%20mount%20the%20shared%20folder%3A

- share host VPN to virual linux
  network 1 --> Host-Only
  network 2 --> NAT

# docker
https://docs.docker.com/get-started/overview/

## docker install
- docker
https://docs.docker.com/engine/install/debian/
- nvidia docker
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker

- (docker env)I am using Ubunu 20.04 and I found daemon.json in /var/snap/docker/current/config/daemon.json
https://stackoverflow.com/questions/43689271/wheres-dockers-daemon-json-missing 
issue: standard_init_linux.go:228: exec user process caused: exec format error

## docker usage
```bash
# list local docker images
sudo docker images
# test docker status by demo
sudo docker run hello-world
# show docker env
sudo docker info
# restart docker env
sudo systemctl restart docker
# delete local images
docker system prune -a --volumes https://stackoverflow.com/questions/44785585/docker-how-to-delete-all-local-docker-images
```

## dockerfile
```dockerfile
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.4.3
FROM ${BASE_IMAGE}
COPY ./packages/ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source ${ROS_ROOT}/setup.bash' >> /root/.bashrc 
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /
```


