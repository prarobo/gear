# GEAR

## Installation
 * Switch to home directory ```cd $HOME```
 * Install git
 	```sudo apt-get install git```
 * Clone the gear repository from github
 	```git clone https://github.com/prarobo/gear.git```
 * Switch to gear catkin workspace ```cd $HOME/gear```
 * Compile ```catkin_make```
 * If you see compilation errors, you might be missing some required packages. Install those packages from ros.
 * Finally add your workspace configuration variables to bashrc
 ```source $HOME/gear/devel/setup.bash```
 If you want this to be persistent, add it to your bashrc
 ```source $HOME/gear/devel/setup.bash>>$HOME/.bashrc```
