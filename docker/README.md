# Under construction  
The development direction is leading towards being able to run all of the software necessary for running the MOAD data collection rig to be run within a docker container to make it as portable as possible. The Dockerfile contained in this folder is still under development but it's goal is to setup all the required libraries for data collection.
Currently MOAD is running on a windows machine, and the docker container is based off of ubuntu 22.04 which is causing some problems, namely for the Canon EDSDK which only recently got Linux support.  
A simple import test program is included to make sure all libraries can be imported successfully.  
  
The ns-docker folder contains scripts for post processing object scan data which are meant to be used **within a NerfStudio Docker Container**, which also makes it easier to get set up with training NeRFs. Instructions for getting started with NerfStudio Docker can be found here:     
<https://docs.nerf.studio/quickstart/installation.html#use-docker-image>  