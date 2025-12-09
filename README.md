# FRA502-LAB-6602
Krit Leetrakul 6602 (Oshi)

# Tree
```
~/ros2_ws/src/lab2/
├── package.xml
├── setup.py
└── lab2/
    ├── __init__.py
    ├── eater.py    # The logic for Turtle 1 (Target)
    └── killer.py   # The logic for Turtle 2 (Chaser)
```
# Start download a file
```
git clone -b <branch> https://github.com/FakeKtOshi/FRA502-LAB-6602.git
```

# Terminal 1
```
colcon build
source install/setup.bash
```

# Terminal 2
```
ros2 service call /spawn_turtle turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
ros2 run lab2 eater.py
```

# Terminal 3
```
ros2 run lab2 killer.py
```
