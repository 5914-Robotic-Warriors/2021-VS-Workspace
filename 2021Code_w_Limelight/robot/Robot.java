/////////////////////////////////////////////////////////////////////
// File: Robot.java
/////////////////////////////////////////////////////////////////////
//
// Purpose: The main file that links every other class and thread
// together, plus some other stuff like.
//
// Environment: Microsoft VSCode Java.
//
// Remarks: Created on 2/25/2020.
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

  // Used for running auto code once.
  boolean autoOnce = true;

  // Toggle for if the drive joystick axes are inverted or not.
  boolean invertDriveToggle = true;

  static DriveThread dthread;
  static boolean drive_thread_active =false;

  // Doubles used for the joystick and analog trigger
  // values in the Mecanum drive deadband.
  double leftXAxisPS, leftYAxisPS, zAxisTriggers;

  // Used for the Mecanum drive deadband.
  final double PS_MEC_DRIVE_DEADBAND = 0.08;

  // Creating the PS controller, with an ID of 0.
  Joystick PS = new Joystick(0);

  Constants constants;
  BallShooter ballShooter;
  static RobotDrive robotDrive;
  DriveThread driveThread;
  WormDrive wormDrive;
  BallIntake ballIntake;
  LimeLightCamera camera;
  // DriveThread driveThread = new DriveThread("driveThread");

  //  Targeting parameters for Limelight camera
  double camera_angle;  //  fixed camera angle from floor in degrees
  double camera_height; //  fixed camera height from floor in inches
  double y_target;      //  Target height from floor in inches.
  double db_theta;      //  deadband for theta positioning
  double db_x;          //  deadband for x positioning 

  int autonomous_init=1;

  @Override
  public void robotInit() {

    constants = new Constants();
    ballShooter = new BallShooter();
    robotDrive = new RobotDrive();
    // driveThread = new DriveThread("name");
    wormDrive = new WormDrive();
    ballIntake = new BallIntake();
    camera=new LimeLightCamera();

    camera_angle = 5.0;
    camera_height = 10.0;
    y_target = 72.0;
    db_theta=1.5; // angle deadband in degrees 
    db_x=2.0;     // x position deadband in inches
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {

    // Put SmartDashboard stuff here, so the SD can get that stuff and the program
    // can use that in autoPeriodic()...
  }

  @Override

  //  This function will be called every 20msec.  Need an 
  //  "init" variable so the thread is created only once.
  public void autonomousPeriodic() {

    if(autonomous_init==1)  {
       dthread=new DriveThread("drive thread");
       autonomous_init=0;
    }

  }

  @Override
  public void teleopPeriodic() {

    wormDrive.adjustShooterAngleManual();
    wormDrive.runClimb();

    // Getting the values of these to be used for Mecanum drive stuff.
    leftXAxisPS = PS.getX();
    leftYAxisPS = PS.getY();
    zAxisTriggers = getZAxisTriggers();

    // If the driver presses the Touchpad button on the PS,
    // change the variable to its opposite state, thus either
    // inverting the drive or un-inverting it.
    if (invertDriveToggle == false) {
      while (PS.getRawButton(constants.PS_TOUCHPAD) == true) {
        invertDriveToggle = true;
      }
    }
    if (invertDriveToggle == true) {
      while (PS.getRawButton(constants.PS_TOUCHPAD) == true) {
        invertDriveToggle = false;
      }
    }

    /*
     * Long if statement that acts as a deadband for the drive. Basically, if the
     * absolute value of X, Y, OR Z axis values are greater than 0.2, run the
     * Mecanum drive. Else, don't run the Mecanum drive stuff. The arguments for
     * this function don't match up with the actual joystick axes for some reason.
     * Depending on the robot, you might have to experiment with these. Z = Right
     * joystick X axis (changed to the analog triggers using getZAxisTriggers()); Y
     * = Left joystick Y axis; X = left joystick X axis. In this case, ySpeed is the
     * strafing stuff, xSpeed is for driving forward/backward, and zRotation is for
     * turning left/right.
     */

    // If either axis is being pressed, run the drive in 1 of 2 ways,
    // depending on the toggle.

    if (((Math.abs(leftXAxisPS) > PS_MEC_DRIVE_DEADBAND) || (Math.abs(leftYAxisPS) > PS_MEC_DRIVE_DEADBAND))) {

      // If the invert drive toggle is false, drive normally.
      if (invertDriveToggle == false) {
        robotDrive.mecanumDrive.driveCartesian(-leftYAxisPS, getZAxisTriggers(), leftXAxisPS);
      }

      // If the toggle is true, the same function but the signs are different.
      else if (invertDriveToggle == true) {
        robotDrive.mecanumDrive.driveCartesian(leftYAxisPS, -getZAxisTriggers(), leftXAxisPS);
      }
    } else {
      robotDrive.mecanumDrive.driveCartesian(0, 0, 0);
    }

    // If the X button on the PS is pressed,
    // and the Square button is NOT pressed...
    if (PS.getRawButton(constants.PS_X_BUTTON) == true) {

      // Shoot balls with front motors at 100% and back motors at 40%.
      ballShooter.ballWheelMove(1.0);

      // Else if the X button on the PS is pressed,
      // and the square button IS pressed...
      /*
       * } else
       * if((PS.getRawButton(constants.PS_X_BUTTON)==true)&&(PS.getRawButton(
       * constants.PS_SQUARE_BUTTON)==true)){
       * 
       * // Run the ball shooter motors backwards at 100% and back motors at 40%.
       * ballShooter.ballShoot(-1,-0.2);
       */
    } if (PS.getRawButton(10) == true) {
      ballShooter.beltWheelMove(-0.1);
    } if (PS.getRawButton(constants.PS_SQUARE_BUTTON) == true) {
      ballShooter.beltWheelMove(0.2);
    } else if((PS.getRawButton(constants.PS_SQUARE_BUTTON) == false) && (PS.getRawButton(constants.PS_X_BUTTON) == false) &&  (PS.getRawButton(10) == false)) {
      // Else, don't run the motors.
      ballShooter.beltWheelMove(0);
      ballShooter.ballWheelMove(0);
    }

    // If the driver pushes the right bumper button on the PS Controller,
    // run the intake motors forward (inwards).
    if (PS.getRawButton(constants.PS_RIGHT_BUMPER)) {

      ballIntake.intakeBalls(-0.25,0.5);

      // Else if the driver pushes the left bumper button on the PS Controller,
      // run the intake motors backwards (outward).
    } else if (PS.getRawButton(constants.PS_LEFT_BUMPER)) {

      ballIntake.intakeBalls(0.25,0.5);

    } else {

      // Else, set the motors to 0 (don't run them).
      ballIntake.intakeBalls(0.0,0.0);

    }
  }

  @Override
  public void testPeriodic() {
  }

  /////////////////////////////////////////////////////////////////////
  // Function: getZAxisTriggers()
  /////////////////////////////////////////////////////////////////////
  //
  // Purpose: Gets the value for the "Z" axis using the 2 analog triggers.
  //
  // Arguments: none
  //
  // Returns: double zAxis: the value from -1 to 1, which is used for
  // controlling the speed and direction for strafing in teleop.
  //
  // Remarks: Created on 2/22/2020.
  //
  /////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////
  public double getZAxisTriggers() {

    // This variable is used for setting the Z axis for strafing.
    double zAxis;

    // These variables are for getting the values for the left and right
    // analog triggers, respectively.
    double leftAnalogTrigger;
    double rightAnalogTrigger;

    // Axes 2 and 3 are the left and right analog triggers, respectively.
    // You have to add 1 because the triggers start at -1 and go to 1.
    // Adding 1 makes them start at 0 when not being pressed.
    leftAnalogTrigger = PS.getRawAxis(3) + 1;
    rightAnalogTrigger = PS.getRawAxis(4) + 1;

    // Do the math for getting the value for strafing.
    // Example 1: if the driver presses the right one down, that value will be 1 - 0
    // = 100% speed (1).
    // Example 2: if the driver presses the left one down, that value will be 0 - 1
    // ; -100% speed (-1).
    zAxis = rightAnalogTrigger - leftAnalogTrigger;

    // Return the value, to be used elsewhere.
    return zAxis;
  }

  ///////////////////////////////////////////////////////////////////
  //  Function:  int align2Target()
  ///////////////////////////////////////////////////////////////////
  //
  //  Purpose:  Using the Limelight camera position the robot
  //            such that the robot aligns itself perpendicular
  //            to the target plane (maximizes available target area)
  //
  //  Arguments: void
  //
  //  Returns: 0 if successful, 1 if not successful.
  //
  //  Remarks:  Direction of strafe needs to be determined
  //  for the sign of the angle returned by the Limelight.
  //  Also might want to add some proportional control to
  //  the speed of the strafe.
  //
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  int align2Target()
  {
    double theta;

    camera.Update_Limelight_Tracking();
    
    theta=camera.get_tx();


    //  Could use a more proportional response here.
    if(theta<-db_theta)  {
      robotDrive.strafeRight(0.5);
    }  else if(theta>db_theta) {
      robotDrive.strafeLeft(0.5);
    } else {
      robotDrive.fullStop();
    }
    return(0);
  }

  ///////////////////////////////////////////////////////////////////
  //  Function:  int position2Target(double distance)
  ///////////////////////////////////////////////////////////////////
  //
  //  Purpose uses the Limelight camera and robotDrive to position
  //  the robot "distance" from target.
  //
  //  Arguments:  double distance, the horizontal distance from the
  //  camera lens to the target.  Units are inches
  //
  //  Returns:  zero at this time, may return a non-zero value in the
  //  event of an error.
  //
  ///////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////
  int position2Target(double distance)
  {
    double delta_y;
    double x;
    double delta_x;

    delta_y=y_target-camera_height;

    camera.Update_Limelight_Tracking();

    x=camera.get_x_distance(camera_angle, delta_y);

    if(x<distance)  {
      delta_x=distance-x;
      if(delta_x>db_x)  {
        robotDrive.driveFwd(delta_x/12.0);  //  argument is in feet
      }  else  {
        robotDrive.fullStop();
      }
    }  else if(x>distance)  {
      delta_x=x-distance;
      if(delta_x>db_x)  {
        robotDrive.driveBwd(delta_x/12.0);  //  argument is in feet
      } else {
        robotDrive.fullStop();
      }
    }
    return(0);
  }

}