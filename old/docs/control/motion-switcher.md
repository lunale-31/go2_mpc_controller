# Motion Switcher

The Go2 robot can be controlled in two different modes: High- and low-level control.
- In [high-level control](high-level.md), you can use pre-defined motion patterns like walking, standing up, or sitting down. 
This mode relies on the controller built into the robot.
- In [low-level control](low-level.md), you can directly interface with the joint motors.
This mode requires to have the inbuilt controller of the robot to bee turned off, as it will otherwise interfere with your inputs.

## Switching the Inbuilt Controller On or Off
The inbuilt motion controller can be controlled via the [Motion Switcher Service Interface](https://support.unitree.com/home/en/developer/Motion%20Switcher%20Service%20Interface).
We provide the [`MotionSwitcher` class](../../src/interface/MotionSwitcher.h) as a wrapper for this.
In Unitree's terminology, the inbuilt controller can be _turned silent_ (`MotionSwitcher::set_silent(true)`) to disable it, or _unsilenced_ (`MotionSwitcher::set_silent(false)`) to reenable it.
For this change to take effect, the robot must be restarted (i.e., powered off and back on entirely).
If the inbuilt controller is silenced, the robot will _not_ stand up when turned on.
If it does stand up on its own, the controller is active.

For convenience, we provide a tool to switch the inbuilt controller as part of this codebase.
After compiling, run the following command to turn the inbuilt controller off:
```bash
./motion-switcher off
```

To reenable the motion controller, run:
```bash
./motion-switcher on
```

Remember to power the robot off and back on for the change to take effect!