# State machine for state management and ego planner trajectory command publish
```
git clone https://github.com/DongnanHu6556/px4_ego.git
cd px4_ego
colcon build
source install/setup.bash
ros2 run px4_ego_py offboard_control_test
```
The state machine runs in munual control at first. Then we can use keyboard to switch different states:

```
cd px4_ego
python3 mode_key.py 
```
- 't' means "takeoff"
- 'p' means "hover at current position (position mode)"
- 'o' means "switch to offboard mode. (If there is no trajectory command, it will return to position mode)"
- 'l' means "land"
- 'd' means "disarm"
When you input 't', the drone will auto takeoff.
<img width="1830" height="1042" alt="takeoff_screenshot" src="https://github.com/user-attachments/assets/883905a2-4168-4a9b-98cc-df09a062ec74" />
