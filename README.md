# SCITOS

## Building
```bash
catkin build
```

**To export compile_commands.json**
```bash
catkin build -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```
_Make sure to copy `compile_commands.json` from under build to project root!_
```bash
# Execute at catkin_ws
jq -s 'map(.[])' build/**/compile_commands.json > ./src/scitos_slam_group_2/compile_commands.json
```

## Running
```bash
roslaunch scitos_annus_loomets_kitsing scitos_autonomous.launch

rosrun scitos_annus_loomets_kitsing teleop_key.p  # To drive with keyboard
```

## Serve slides
```bash
marp -s docs
```
