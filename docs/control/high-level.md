# High-level Control

In the high-level control mode, the robot is controlled by an inbuilt controller which can be tasked to run high-level commands like standing up, sitting down, or walking.
This requires the internal controller to be turned on (see [this document](motion-switcher.md) for details).

In Unitree's terminology, the high-level control is called the [Sports Services Interface](https://support.unitree.com/home/en/developer/sports_services).
We provide a wrapper to some high-level commands in the [HighLevelControl class](../../src/interface/HighLevelControl.h).
See [stand-up.cpp](../../src/entrypoints/stand-up.cpp) for an example of how to use that wrapper.

To extend the HighLevelControl class, it can be helpful to look into [Unitree's Python SDK](https://github.com/unitreerobotics/unitree_sdk2_python/tree/master/unitree_sdk2py/go2/sport) to find out the API IDs and parameter format for different commands.