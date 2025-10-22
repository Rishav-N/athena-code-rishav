# To use these message interfaces in your own package:

## Python

### In your python node:
```
from umdloop_theseus_can_messages.msg import CANA
```

or 
```
from umdloop_theseus_can_messages.msg import CANB
```

### package.xml
```
<exec_depend>umdloop_theseus_can_messages</exec_depend>
```

## C++

### In your C++ node:
```
#include umdloop_theseus_can_messages/msg/CANA
```

or
```
#include umdloop_theseus_can_messages/msg/CANB
```

### CMakeLists.txt
```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(umdloop_theseus_can_messages REQUIRED)

add_executable(some_node src/node_code_file.cpp)
ament_target_dependencies(some_node rclcpp umdloop_theseus_can_messages)

install(TARGETS
  some_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

### package.xml
```
<depend>umdloop_theseus_can_messages</depend>
```

## Confirm access to message interfaces:
From your workspace root:
```
source /opt/ros/humble/setup.bash
colcon build --packages-select umdloop_theseus_can_messages
source install/setup.bash
```

then
```
ros2 interface show umdloop_theseus_can_messages/msg/CANA
```

Should return
```
uint16 id
uint8[] data
```
