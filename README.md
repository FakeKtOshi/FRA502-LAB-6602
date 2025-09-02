# FRA502-LAB-6602
Krit Leetrakul 6602 (Oshi)

****THESE STEPS ARE FOR LAB3****

HOW TO RUN WITH TERMINAL:
STEP1 : ros2 run turtlesim_plus turtlesim_plus_node.py 
STEP2 : ros2 run lab3 eater.py
STEP3 : ros2 run lab3 killer.py 
STEP4 : ros2 service call /max_pizza controller_interfaces/srv/SetMaxPizza "max_pizza:
  data: 10
log:
  data: ''" 
*Feel free to change a data based on your needs.


HOW TO RUN WITH LAUNCH STEP:
STEP1 : colcon build
STEP2 : . install/setup.bash
STEP3 : ros2 launch lab3 lab3_bringup.launch.py 

********************************
