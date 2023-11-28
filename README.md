### Hardware
- Plug in all peripherals to the Jetson
	- CANable, lidar, camera, monitor (if not powered by external supply or battery), mouse, and keyboard
	- Recommended to use bluetooth mouse and keyboard 
- Connect either power supply or battery with barrel adapter to the Jetson
	- The Jetson will start to boot up, it may take some time
- Connect the LiFePo4 battery to the Power Distribution Panel (PDP) with the red Anderson connector
- Press down on red button on the circuit breaker and push in the small black latch
	- [This](https://www.andymark.com/products/120-amp-breaker) is the circuit breaker if you are unsure

There should now be flashing orange lights on the PDP. If it is flashing red then check the green and yellow wires, these lights mean that there is no CAN bus communication detected. CAN-High is represented by the yellow wire, while CAN-Low is represented by the green wire.

### Software
Launch the test bot and slam_toolbox:
```bash
ros2 launch lunabot_bringup test_bot_navigation.launch.py
```
- Wait for RViz to finish loading before launching Navigation2
Launch Navigation2
```bash
ros2 launch nav2_bringup navigation_launch.py
```
- No need to set an initial pose, it's already set at (0, 0)
- Select "Nav2 Goal" in RViz and choose the goal's position and orientation with the arrow