# Project 1 - Task 3
### Lombe

### **Compiling and running du_chileshe_lombe**

1. Prior to starting, make sure ```roscore``` and ```turtlesim_node```
are running.

2. Place the project in your catkin workspace
```bash
cd ~/catkin_ws/src/du_chileshe_lombe/
```
Alternatively, you can pull it from github
```bash
    git clone https://github.com/LombeC/SAIR-Project-1.git
```
3. Navigate to the catkin folder and build the project
```bash
cd ~/catkin_ws 	# Navigate to the catkin_ws
catkin_make 	# Build
```
4. Make sure the build executable is on your path. Execute the following command while inside *catkin_ws* :

```bash
source ./devel/setup.bash
```

5. Lastly run the executable 
```bash
rosrun du_chileshe_lombe du_chileshe_lombe
```
You should expect to see the turtle draw the letter D.