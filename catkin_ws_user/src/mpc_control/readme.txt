This package implements MPC solver for car to follow given points.

Dependencies & Installation:

The packages needs, IPOPT, CPPAD and Eigen(included in src) libraries. 

The installation needs some sace in /media/boot folder of odroid, it was allocated to 10MB in previous versions - Reflash the SD card with new image for 100MB+ boot space. 

Installation of IPOPT. 
- Remove the ASL folder from Thirdparty libraries to install IPOPT on Model Car. The ASL Library does not have few definitions for ARM Architecture.

sudo apt-get install gfortran
apt-get install unzip
#Check for latest version available on the website.
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.8.zip && unzip Ipopt-3.12.8.zip && rm Ipopt-3.12.8.zip

#install_ipopt.sh can be found in the mpc_control folder. 
bash install_ipopt.sh Ipopt-3.12.8


Installation of CPPAD 
- The package from apt-get gives errors for ARM architecture, install the newst from the CPPAD src. 

Follow the instructions on the following website for installation. 

https://www.coin-or.org/CppAD/Doc/install.htm

(If cmake with options doesnt work, just try cmake .. in build- It does default installation and generally it is sufficient)





equations for car model - ??

veolcity - mt_ps from odometry.

calculate min max acceleration - 

1000rpm for motor - then 
wheel speed max =  1000/(5.5*60) = 3.03rot per sec => 3.03*2*3.14*0.031 = 0.5899 mps

if 1000rpm is wheel speed then 
wheel speed max  =  1000/60 ==> 3.244 mps. 

if 3.24 mps is max velocity, whats the time I want to attain it?

lets say in 5s then -> 3.24/5 = 0.65 mpss

min max accc =  [-0.65, 0.65] lets limit it to [-0.5:0.5]


1m - 5.1366 rotations of wheel.


acceleration of in rotations =  [-2.568:2.568 rot/ss]

converting acc from mps to rot per sec - > 

Hows the speed equation ?? 


//control loop running at 10Hz then 
0.05mps is increase in veocity every time 


