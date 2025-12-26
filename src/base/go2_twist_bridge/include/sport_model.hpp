#ifndef _SPORT_MODEL_
#define _SPORT_MODEL_
#include <iostream>

#pragma pack(1)

// 1001: Damping Control
const int32_t ROBOT_SPORT_API_ID_DAMP = 1001;

// 1002: Balance Standing: Control the robot to enter a balanced standing state, maintaining an upright and stable position.
const int32_t ROBOT_SPORT_API_ID_BALANCESTAND = 1002;

// 1003: Stop movement. Stop all movement of the robot and bring it to an immediate stop.
const int32_t ROBOT_SPORT_API_ID_STOPMOVE = 1003;

// 1004: Standing up, controlling the robot to return to a standing position from other postures (such as sitting or lying down).
const int32_t ROBOT_SPORT_API_ID_STANDUP = 1004;

// 1005: Standing descent, controlling the robot to change from a standing posture to other postures (such as sitting or lying down).
const int32_t ROBOT_SPORT_API_ID_STANDDOWN = 1005;

// 1006: Restore standing posture; control the robot to return to a standing posture after losing balance.
const int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND = 1006;

// 1007: Euler angle control, used to adjust the robot's attitude (such as pitch, roll, yaw).
const int32_t ROBOT_SPORT_API_ID_EULER = 1007;

// 1008: Movement; controlling the robot to move, which may be linear movement or turning.
const int32_t ROBOT_SPORT_API_ID_MOVE = 1008;

// 1009: Sit down, control the robot to enter a sitting posture.
const int32_t ROBOT_SPORT_API_ID_SIT = 1009;

// 1010: Restore the robot from a sitting position to a standing position.
const int32_t ROBOT_SPORT_API_ID_RISESIT = 1010;

// 1011: Switch gait, switch the robot's gait (such as walking, running, crawling, etc.).
const int32_t ROBOT_SPORT_API_ID_SWITCHGAIT = 1011;

// 1012: Trigger, which may be used to trigger a specific action or event.
const int32_t ROBOT_SPORT_API_ID_TRIGGER = 1012;

// 1013: Body height adjustment, adjust the robot's body height.
const int32_t ROBOT_SPORT_API_ID_BODYHEIGHT = 1013;

// 1014: Adjust the height of the robot's feet.
const int32_t ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT = 1014;

// 1015: Speed ​​level adjustment, adjusts the robot's movement speed level.
const int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL = 1015;

// 1016: Greet the robot by controlling it to perform a "greeting" action.
const int32_t ROBOT_SPORT_API_ID_HELLO = 1016;

// 1017: Stretch, control the robot to perform the "stretch" action.
const int32_t ROBOT_SPORT_API_ID_STRETCH = 1017;

// 1018: Track following, controlling the robot to move along a predetermined trajectory.
const int32_t ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW = 1018;

// 1019: Continuous gait, controlling the robot to perform continuous gait movements.
const int32_t ROBOT_SPORT_API_ID_CONTINUOUSGAIT = 1019;

// 1020: Content, which may be used to control the robot to display or perform some content (such as playing video or audio).
const int32_t ROBOT_SPORT_API_ID_CONTENT = 1020;

// 1021: Rolling, control the robot to perform a "rolling" or "tumbling" action.
const int32_t ROBOT_SPORT_API_ID_WALLOW = 1021;

// 1022: Dance 1, control the robot to perform the first dance move.
const int32_t ROBOT_SPORT_API_ID_DANCE1 = 1022;

// 1023: Dance 2, control the robot to perform the second dance move.
const int32_t ROBOT_SPORT_API_ID_DANCE2 = 1023;

// 1024: Get body height, get the robot's current body height.
const int32_t ROBOT_SPORT_API_ID_GETBODYHEIGHT = 1024;

// 1025: Get the height of the raised foot, obtain the current height of the robot's raised foot.
const int32_t ROBOT_SPORT_API_ID_GETFOOTRAISEHEIGHT = 1025;

// 1026: Get speed level, get the robot's current movement speed level.
const int32_t ROBOT_SPORT_API_ID_GETSPEEDLEVEL = 1026;

// 1027: Switch joysticks to change the control mode or function of the joystick.
const int32_t ROBOT_SPORT_API_ID_SWITCHJOYSTICK = 1027;

// 1028: Attitude, controlling the robot to enter a specific posture or pose.
const int32_t ROBOT_SPORT_API_ID_POSE = 1028;

// 1029: Scraping, controlling the robot to perform the "scraping" action.
const int32_t ROBOT_SPORT_API_ID_SCRAPE = 1029;

// 1030: Front somersault, control the robot to perform a front somersault.
const int32_t ROBOT_SPORT_API_ID_FRONTFLIP = 1030;

// 1031: Jump forward, control the robot to perform a jump forward action.
const int32_t ROBOT_SPORT_API_ID_FRONTJUMP = 1031;

// 1032: Pounce forward, control the robot to perform a pounce forward action.
const int32_t ROBOT_SPORT_API_ID_FRONTPOUNCE = 1032;

const int32_t ROBOT_SPORT_API_ID_CLASSICWALK = 2049;
#endif
