Welcome to the FRC6193-2018-3Cim2Speed-Java wiki!
# Introduction
Code related to FRC 2017 rebuilt robot to test a 3 CIM dual speed gearbox. Competition with 2017 field to be held on October 14 in Midland Mi.
# Team Information
Team FRC 6193 from Bay City Michigan at All Saints High School. Team size is 15 students and 6 mentors.
# Code Information
Robot design is to handle 2017 game. The following base robot design is:
* Gear grabber that picks up gears off the ground with a spinning pincher that lifts the gear off the ground.
    * This requires a spinner motor and lift motor.
    * Current from the motor is used to tell if the spinner pinched a gear and the lift motor has hit the vertical position.
* Rope Climber that is a 36:1 Vex Robotics gearbox connected to a CIM with a 1/2 hex shaft with Velcro.
    * If you know the game you might understand this.
* Driveline gearbox that contains a Vex Robotics 3 CIM ball shifter
    * 3.68 Ratio Spread [60:24]
    * 3rd Stage [60:24]
    * Using JVN-DesignCalc the speeds are a calculated 4.43 and 16.28 ft/s
    * Redesigned RobotDrive class to handle
        * 3 CIMs with the 3rd CIM as a miniCIM
        * Automatic gear shifting for up-shift at ~6000 rpm and down-shift at ~50 rpm.
        * Manual shifting mode.
        * Arcade style of Drive with triggers for forward and reverse
        * Pneumatics that control the solenoids for shifting 
* Initial Testing
    * High speed was at the limit of uncontrollable by the students. Perfect.
    * Low speed was usable and helped with turning response.
    * Low speed current was <40 Amps pushing solid surface.
    * MiniCIM added a noticeable pushing force against a large adult attempting to stop the robot.



