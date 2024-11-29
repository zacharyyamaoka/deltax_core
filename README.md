
Meta Package with Code to run Delta X robot in ROS2 


Reference Links:

https://docs.deltaxrobot.com/reference/gcodes/useful_commands/ for useful commands

https://docs.deltaxrobot.com/reference/gcodes/gc_xs_v5/ for gcode reference


Code based on:

https://github.com/VanThanBK/deltax_ros_public
https://github.com/VanThanBK/python-deltax/tree/master
https://github.com/deltaxrobot/deltax_pick_place_script


From: https://github.com/VanThanBK/Delta-X-Firmware/blob/master/Delta_Firmware/Geometry.h


//                                          / \             RD_RF    
//                                         /   \             /
//                                        /     \ Theta2    /                                   ^ Y   ^ Z
//                                       /       \ ________/____                                |    /
//                           Theta3     /         \            /                                |   /
//                                     /     *O    \          /                                 |  /
//                        RD_F_______ /             \        /                                  | /
//            ^                      /               \      /                                 __|/_____________>
//            |                      -----------------     /                                    |O             X
//            |                           Theta1          /
//            |                                          /_______RD_RE
//            |                                         /
//            |                                        /
//            |                                       /
//            |                                 / \  /
//            |                      RD_E_____ /   \/
//     RD_B   |                               /  *  \     RD_U
//            |                               -------    /
//            |                                  |      /
//            |                     RD_W________ |_____/_
//            |                                  |
//            |                                  | _______RD_V
//            |                                  ^
//____________|__________________________________________________________________________________________________________
