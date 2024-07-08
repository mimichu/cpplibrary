# cpplibrary
Collection of utilities for c++.

Dependencies:
* [[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)]
* [[yaml-cpp](https://github.com/jbeder/yaml-cpp)]

# Install
```
$ mkdir build & cd build
$ cmake ..
$ make
$ sudo make install
```

# Usage
1. Install.
2. In your code, include the header files. For example:
    ```
    #include <RobotUtilities/utilities.h>
    #include <RobotUtilities/TimerLinux.h>
    ```
3. If you use a binary library, link the library to your program in CMakeLists.txt. For example:
    ```
    find_library(TIMER_LIB TimerLinux HINTS /usr/local/lib/RobotUtilities)
    find_library(RobotUtilLib Utilities HINTS /usr/local/lib/RobotUtilities)
    target_link_libraries(forcecontrol_node
      ${TIMER_LIB}
      ${RobotUtilLib}
    )
    ```

# Rules
* Only source files.
* Platform specific code should be put in separate folders, e.g. ```/win/```
