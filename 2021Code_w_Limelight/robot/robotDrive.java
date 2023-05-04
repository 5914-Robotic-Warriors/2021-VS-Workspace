/////////////////////////////////////////////////////////////////////
//  File:  RobotDrive.java
/////////////////////////////////////////////////////////////////////
//
//  Purpose:  The intent is to create a class entirely devoted to
//            autonomous operation of the robot mecanum drive.
//            Although variable names may be identical to other
//            class definitions, the use of "private" should make
//            this an independent set of variables and functions.
//            It is expected that this class will be allocated in
//            a thread in autonomous.
//
//  Compile Environment:  Java via Microsoft VS
//
//  Inception Date:  Originally code developed in early 2020 but
//            heavily modified for said purpose.
//
//  Revisions:
//
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;


class RobotDrive extends Robot {

     /*
     * Drive encoders are on the NEO drive motors. The output from the encoders is 
     * 42 counts per shaft revolution.  A gear reduction of 12.75:1 implies 12.75  
     * times 42 per revolution of the output shaft (535.5 counts per output shaft
     * revolution).
     * An 8 inch wheel diameter implies a distance traveled of PI*8.0 = 25.13 inches. 
     * Distance resolution of the encoder/gearbox combination is 25.13/535.5 = 0.0352 
     * inches per count. 
     *  
     * Encoder position is read from a CANEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     * So, one revolution of the motor implies 42 counts or 0.0352*42 inches per
     * revolution, e.g., 1.478 inches per unit returned.
     */
    //SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
     
    private final double ENCODER_RESOLUTION = 1.478; // inches per output value

    // Fixed parameters for conversion of distance to encoder counts.
    //private final double WHEEL_DIAMETER = 8.0;
    //private final double INCHES_PER_FOOT = 12.0;
    //private final double CM_PER_METER = 100.0;

    // Fixed parameters for driveFwd(...)/driveBwd(...)
    private final double START_SPEED = 0.1; // Also used in acceleration functions.
    private final double MAX_SPEED = 0.6;
    private final double MIN_SPEED = 0.1;
    private final double BRAKE_SPEED = 0.3;
    private final double BRAKE_FRACTION = 0.4;

    // Fixed parameters for console updates and while() loop escapes.
    //private final int ENC_CONSOLE_UPDATE = 20;
    //private final int ENC_LOOP_ESCAPE = 250;
    private final int GYRO_CONSOLE_UPDATE = 20;
    private final int GYRO_LOOP_ESCAPE = 200;

    // Fixed parameters for gyro operation. Specified here to facilitate
    // changes without confusion in the various functions using these
    // variables.
    private final double ROT_SPEED = 0.5; // Starting rotation speed for turning
    // As we approach the target we reduce the speed by this factor
    private final double ROT_ATTEN = 1.5;
    // proximity (in degrees) to target angle stage 1 rotation attenuation rate
    private final double ANGL_PROX_1 = 25.0;
    // proximity (in degrees) to target angle stage 2 rotation attenuation rate
    private final double ANGL_PROX_2 = 5.0;

    // Initial drive encoder position.
    public double initDrivePosition;

    // Initial drive gyro angle.
    public double initDriveGyroAngle;


    // Creating the drive motors.
    CANSparkMax frontLeftDriveMotor, backLeftDriveMotor, frontRightDriveMotor, backRightDriveMotor;
    MecanumDrive mecanumDrive;
    private Delay delay;

    CANEncoder frontLeftEncoder;
    ADXRS450_Gyro driveGyro;

 
    
  
    //  Constructor for MecDrive()
    RobotDrive()  {

        delay=new Delay();

        // Creating the MecanumDrive constructor, which links all 4 motors together.
        
        // Constructing the motors, giving them their IDs, and making them brushless.
        frontLeftDriveMotor = new CANSparkMax(constants.FRONT_LEFT_DRIVE_MOTOR_ID, MotorType.kBrushless);
        backLeftDriveMotor = new CANSparkMax(constants.BACK_LEFT_DRIVE_MOTOR_ID, MotorType.kBrushless);
        frontRightDriveMotor = new CANSparkMax(constants.FRONT_RIGHT_DRIVE_MOTOR_ID, MotorType.kBrushless);
        backRightDriveMotor = new CANSparkMax(constants.BACK_RIGHT_DRIVE_MOTOR_ID, MotorType.kBrushless);

        mecanumDrive = new MecanumDrive(frontLeftDriveMotor, backLeftDriveMotor, frontRightDriveMotor,
            backRightDriveMotor);

        // Encoder for the front left drive motor.
        // Used for the auto functions.
        frontLeftEncoder= frontLeftDriveMotor.getEncoder(EncoderType.kHallSensor, 42);

        // Gyro for auto drive functions.
        driveGyro = new ADXRS450_Gyro(); 


        // Set the drive motors to coast mode to help prevent tipping,
        // and to make the drive less jerky.
        frontLeftDriveMotor.setIdleMode(IdleMode.kCoast);
        frontRightDriveMotor.setIdleMode(IdleMode.kCoast);
        backLeftDriveMotor.setIdleMode(IdleMode.kCoast);
        backRightDriveMotor.setIdleMode(IdleMode.kCoast);

        // We want to establish an initial encoder reading. This will enable resetting
        // encoder position to zero when we start moving. We use absolute values to
        // make the subsequent subtraction more easily interpreted.
        frontLeftEncoder.setPosition(0.0);
        frontLeftEncoder.getPosition(); // should be zero
        initDriveGyroAngle = driveGyro.getAngle();


    }

    /////////////////////////////////////////////////////////////////////
    // Function: strafeLeft(...)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for strafing left, without encoder magic.
    //
    // Arguments: double strafeSpeed
    //
    // Returns: void
    //
    // Remarks: Created on 3/07/2020.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public void strafeLeft(double strafeSpeed) {

        // Strafe left at the input speed.
        frontLeftDriveMotor.set(-strafeSpeed);
        backLeftDriveMotor.set(-strafeSpeed);
        frontRightDriveMotor.set(strafeSpeed);
        backRightDriveMotor.set(strafeSpeed);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: strafeRight(...)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for strafing right, without encoder magic.
    //
    // Arguments: double strafeSpeed
    //
    // Returns: void
    //
    // Remarks: Created on 3/07/2020.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public void strafeRight(double strafeSpeed) {

        // Strafe right at the inputted speed.
        frontLeftDriveMotor.set(strafeSpeed);
        backLeftDriveMotor.set(strafeSpeed);
        frontRightDriveMotor.set(-strafeSpeed);
        backRightDriveMotor.set(-strafeSpeed);
    }


    /////////////////////////////////////////////////////////////////////
    // Function: strafeLeftAuto(...)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for strafing left in autonomous.
    //
    // Arguments: double speed, double time (how long to strafe in seconds).
    //
    // Returns: void
    //
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public void strafeLeftAuto(double speed, double time) {

        // Strafe at the inputted speed.
        frontLeftDriveMotor.set(-speed);
        backLeftDriveMotor.set(-speed);
        frontRightDriveMotor.set(speed);
        backRightDriveMotor.set(speed);

        // Run the motors and stop after these many seconds.
        delay.delay_seconds(time);

        // Stop the motors after "time" amount of seconds.
        frontLeftDriveMotor.set(0);
        backLeftDriveMotor.set(0);
        frontRightDriveMotor.set(0);
        backRightDriveMotor.set(0);

    }

    /////////////////////////////////////////////////////////////////////
    // Function: strafeRightAuto(...)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for strafing right in autonomous.
    //
    // Arguments: double speed, double time (how long to strafe in seconds).
    //
    // Returns: void
    //
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public void strafeRightAuto(double speed, double time) {

        // Strafe at the inputted speed.
        frontLeftDriveMotor.set(speed);
        backLeftDriveMotor.set(speed);
        frontRightDriveMotor.set(-speed);
        backRightDriveMotor.set(-speed);

        // Run the motors and stop after these many seconds.
        delay.delay_seconds(time);

        // Stop the motors after "time" amount of seconds.
        frontLeftDriveMotor.set(0);
        backLeftDriveMotor.set(0);
        frontRightDriveMotor.set(0);
        backRightDriveMotor.set(0);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: public int driveFwd(double distance)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Drives the robot forward the distance in feet specified
    // by the argument.
    //
    // Arguments:double distance, The distance to be traveled in feet.
    //
    // Returns: A double representing overshoot/undershoot of the movement
    // in inches.
    //
    // Remarks: 02/09/2020: Modified to include acceleration/deceleration
    // 02/11/2020: Noted that the inequality regarding fraction
    // should have been '>' vs. '<'
    // Increased delays within while() loops
    // Reduced print statements to make it easier
    // to determine position and fraction.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public double driveFwd(double distance) {
        double counts; // Encoder counts.
        double initial_position; // Our initial position.
        double current_position; // CUrrent position of the robot.
        double target; // Init position + encoder counts.

        double fraction; // How close we are to our target.
        double heading; // Initial gyro angle.

        double error; // Overshoot/undershoot.

        // Determine where we are pointing - we want to maintain this heading during
        // this forward movement.
        heading = driveGyro.getAngle();
        frontLeftEncoder.setPosition(0.0);

        // Read encoder #1, get the initial encoder position and assign it to
        // the current position. Calculate the number of encoder counts
        // necessary to reach the final destination (target).
        initial_position = frontLeftEncoder.getPosition(); // should be zero

        System.out.println("initPos = " + initial_position);

        current_position = initial_position;
        counts = calcCounts_SAE(distance);
        target = initial_position + counts;

        // fraction starts out as equal to 1.0 and decreases as we approach the target.
        // fraction is counts remaining divided by total counts.
        fraction = Math.abs((target - current_position) / (target - initial_position));

        System.out
                .println("initial_position = " + initial_position + " target = " + target + " fraction = " + fraction);

        // We attempt a bit of proportional control as we approach the target. We want
        // to slow down so that we don't overshoot. These fractions appear to work.
        // We drive at high speed for 80% of the distance and then slow. On carpet this
        // seemed to work very well for distance of 10 feet.
        // We want braking enabled.
        // We also need to put a timer within the while() loop to provide an escape in
        // the event that the system gets lost during autonomous requiring a restart of
        // the program.

        // Get moving and accelerate to MAX_SPEED.
        if (current_position < target) {
            accelerateFwd();
        }
        System.out.println("Acceleration complete");

        // Monitor position and continue to travel at max speed until
        // fraction<BRAKE_FRACTION. Note that fraction starts out at 1.0
        // and decreases as we approach the target encoder value.
        // 02/11/2020: Changed the sign of the inequality to '>'
        // Changed the delay from 0.01 to 0.02
        // Moved the position update outside of the while() loop
        while ((current_position < target) && (fraction > BRAKE_FRACTION)) {
            moveFwd(MAX_SPEED, heading);
            current_position = frontLeftEncoder.getPosition();
            fraction = Math.abs((target - current_position) / (target - initial_position));
            delay.delay_seconds(0.02);
        }

        // Where are we?
        System.out
                .println("current_position = " + current_position + " target = " + target + " fraction = " + fraction);
        System.out.println("Decelerating");
        // Ok, we should be at the braking fraction. Time to decelerate to BRAKE_SPEED.
        decelerateFwd();

        // Continue at BRAKE_SPEED until we reach the target encoder value
        // 02/11/2020: Changed the delay from 0.01 to 0.02
        // 02/11/2020: Moved the position update outside of the while() loop
        while (current_position < target) {
            moveFwd(BRAKE_SPEED, heading);
            delay.delay_seconds(0.02);
            current_position = frontLeftEncoder.getPosition();
        }

        System.out
                .println("current_position = " + current_position + " target = " + target + " fraction = " + fraction);

        // stop movement, compute error (overshoot or undershoot)
        mecanumDrive.driveCartesian(0, 0, 0);
        current_position = frontLeftEncoder.getPosition();
        error = calcDistance_SAE(current_position - target);
        System.out.println("error = " + error + " inches");

        return (error);
    }

    ///////////////////////////////////////////////////////////////////////////
    // Function: void moveFwd(double speed,double target)
    ///////////////////////////////////////////////////////////////////////////
    //
    // Purpose: Uses on_board gyro to drive the robot straight forward
    // given a target angle as an argument.
    // Requires use of the arcadeDrive function with the first
    // argument being the forward speed and the second being
    // the turn applied.
    //
    // Arguments: double speed. Must be between -1.0 and 1.0.
    // double heading - the target angle.
    //
    // Returns: void
    //
    // Remarks: This function will be called every 20 msec. Updating
    // the position and heading each time it is called will
    // overwhelm the system.
    // 02/11/2020: Commented out the print statement.
    //
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    public void moveFwd(double speed, double heading) {

        double corr = 0.2;
        double angle = 0;
        double delta; // The difference between the target and measured angle

        angle = driveGyro.getAngle();

        delta = angle - heading;

        // According to the documentation for DifferentialDrive.arcadeDrive(speed,turn)
        // the arguments are squared to accommodate lower drive speeds. If for example
        // the gain coefficient is 0.05 and the angle error is 5 degrees, the turning
        // argument would be 0.25*0.25 = 0.0625. This is a pretty slow correction.
        // We needed a larger correction factor - trying 0.2 for now. The range for
        // the turn is -1.0 to 1.0. Positive values are said to turn clockwise,
        // negatives counterclockwise.
        mecanumDrive.driveCartesian(0, speed, -corr * delta);

        // System.out.println(" heading = " + heading + " angle = " + angle + " delta =
        // " + delta);

    }

    /////////////////////////////////////////////////////////////////////
    // Function: accelerateFwd( ... )
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for accelerating the robot with driveFwd().
    //
    // Arguments: double heading, i.e., the direction of travel.
    //
    // Returns: void
    //
    // Remarks: The listed speed increment and delay will allow
    // approximately 2 seconds to accelerate from a
    // speed of 0.1 to 0.6. We may want to alter these
    // variables depending on the distance.
    //
    // 02/11/2020: Simplified the acceleration phase of
    // the movement. Eliminated reading of angle and
    // direction correction. This allowed eliminating
    // the "heading" argument.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    void accelerateFwd() {
        double speed = 0.1;

        while (speed < MAX_SPEED) {
            mecanumDrive.driveCartesian(0, speed, 0);
            delay.delay_seconds(0.02);
            speed += 0.01;
        }
    }

    /////////////////////////////////////////////////////////////////////
    // Function: decelerateFwd( ... )
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for decelerating the robot with driveFwd().
    //
    // Arguments: double heading, i.e., the direction of travel
    //
    // Returns: void
    //
    // Remarks: The listed speed increment and delay will allow
    // approximately 2 seconds to decelerate from a
    // speed of 0.1 to 0.6. We may want to alter these
    // variables depending on the distance.
    //
    // 02/11/2020: Simplified the deceleration phase of
    // the movement. Eliminated reading of angle and
    // direction correction.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    void decelerateFwd() {
        double speed = MAX_SPEED;

        while (speed > MIN_SPEED) {
            mecanumDrive.driveCartesian(0, speed, 0);
            delay.delay_seconds(0.02);
            speed -= 0.01;
        }
    }

    /////////////////////////////////////////////////////////////////////
    // Function: public int driveBwd(double distance)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Constructor the the class. Creates the devices used to
    // control the robot drive system.
    //
    // Arguments:double distance, The distance to be traveled in inches.
    //
    // Returns: An int representing overshoot/undershoot of the movement
    // in inches.
    //
    // Remarks: It is assumed that the first time a movement is attempted
    // that it is a forward movement. The first movement appears
    // to determine the direction of the encoder counts. What
    // we observed during the last practice was that when we
    // started the program and the first motion was backwards
    // we saw increasing encoder counts. Assumed here is that
    // if the first movement on program start is forward then
    // reversing direction should create decreasing encoder
    // counts.
    // A possible option is to use the other encoder for reverse
    // motion. We shall see..
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public double driveBwd(double distance) {
        double counts; // Encoder counts.
        double initial_position; // Our initial position.
        double current_position; // CUrrent position of the robot.
        double target; // Init position + encoder counts.

        double fraction; // How close we are to our target.
        double heading; // Initial gyro angle.

        double error; // Overshoot/undershoot.

        // Determine where we are pointing - we want to maintain this heading during
        // this forward movement.
        heading = driveGyro.getAngle();
        frontLeftEncoder.setPosition(0.0);

        // Read encoder #1, get the initial encoder position and assign it to
        // the current position. Calculate the number of encoder counts
        // necessary to reach the final destination (target).
        initial_position = frontLeftEncoder.getPosition();
        current_position = initial_position;
        counts = calcCounts_SAE(distance);
        target = initial_position - counts;

        // fraction starts out as equal to 1.0 and decreases as we approach the target.
        // fraction is counts remaining divided by total counts.
        fraction = Math.abs((target - current_position) / (target - initial_position));

        System.out
                .println("initial_position = " + initial_position + " target = " + target + " fraction = " + fraction);

        // We attempt a bit of proportional control as we approach the target. We want
        // to slow down so that we don't overshoot. These fractions appear to work.
        // We drive at high speed for 80% of the distance and then slow. On carpet this
        // seemed to work very well for distance of 10 feet.
        // We want braking enabled.
        // We also need to put a timer within the while() loop to provide an escape in
        // the
        // event that the system gets lost during autonomous requiring a restart of the
        // program.

        // Need to test and determine sign of inequality. We had the curious behavior of
        // increasing encoder counts when moving backwards. It could have been due to
        // the fact that the first motion was backwards in our "spaghetti code". We will
        // Need to play around with the signs to get it working right.
        while (current_position > target) {
            if (fraction > BRAKE_FRACTION) {
                moveBwd(START_SPEED, heading);
            } else {
                moveBwd(BRAKE_SPEED, heading);
            }
            delay.delay_seconds(0.01);

            current_position = frontLeftEncoder.getPosition();
            fraction = Math.abs((target - current_position) / (target - initial_position));
        }

        // stop movement, compute error (overshoot or undershoot)
        mecanumDrive.driveCartesian(0, 0, 0);
        current_position = frontLeftEncoder.getPosition();
        error = calcDistance_SAE(current_position - target);
        System.out.println("error = " + error + " inches");

        return (error);
    }

       /////////////////////////////////////////////////////////////////////
    // Function: driveFwd_Inches(...)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for driving forward in autonomous.
    //
    // Arguments: double inches.
    //
    // Returns: void
    //
    // Remarks: Created on 12/08/2021.  Intended to be used to position
    // the robot based on camera measurements.
    //
    // 
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public void driveFwd_Inches(double inches) {

        double encoder_count;
        double target_count;

        // Initialize the encoder to 0 (reset it).
        frontLeftEncoder.setPosition(0);

        // Our current encoder count reading (should be zero).
        encoder_count = 0;

        // This should give us how many motor revolutions.
        // The NEO motor/SparkMax has a wierd encoder
        // readback format.
        target_count = inches / ENCODER_RESOLUTION;

        while (target_count < encoder_count) {

            // Drive forward.
            frontLeftDriveMotor.set(0.4);
            backLeftDriveMotor.set(-0.4);
            frontRightDriveMotor.set(0.4);
            backRightDriveMotor.set(-0.4);

            // Delay for 20 ms.
            delay.delay_milliseconds(20.0);

            // Read the encoder, and get our current count.
            encoder_count = Math.abs(frontLeftEncoder.getPosition());
        }

        // Stop the motors.
        fullStop();

    }

    /////////////////////////////////////////////////////////////////////
    // Function: driveBwd_Inches(...)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Used for driving reverse in autonomous.
    //
    // Arguments: double inches.
    //
    // Returns: void
    //
    // Remarks: Created on 12/08/2021.  Intended to be used to position
    // the robot based on camera measurements.  It is assumed that
    // negative readings will be realized with reverse motion.
    //
    // 
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public void driveBwd_Inches(double inches) {

        double encoder_count;
        double target_count;

        // Initialize the encoder to 0 (reset it).
        frontLeftEncoder.setPosition(0);

        // Our current encoder count reading (should be zero).
        encoder_count = 0;

        // This should give us how many motor revolutions.
        // The NEO motor/SparkMax has a wierd encoder
        // readback format.
        target_count = inches / ENCODER_RESOLUTION;

        while (target_count > encoder_count) {

            // Drive forward.
            frontLeftDriveMotor.set(-0.4);
            backLeftDriveMotor.set(0.4);
            frontRightDriveMotor.set(-0.4);
            backRightDriveMotor.set(0.4);

            // Delay for 20 ms.
            delay.delay_milliseconds(20.0);

            // Read the encoder, and get our current count.
            encoder_count = Math.abs(frontLeftEncoder.getPosition());
        }

        // Stop the motors.
        fullStop();

    }

    ///////////////////////////////////////////////////////////////////////////
    // Function: void moveBwd()
    ///////////////////////////////////////////////////////////////////////////
    //
    // Purpose: Uses on_board gyro to drive the robot straight backward
    // given a target angle as an argument.
    // Requires use of the arcadeDrive function with the first
    // argument being the forward speed and the second being
    // the turn applied.
    //
    // Arguments: double speed. Must be between -1.0 and 1.0.
    // double angle - the target angle.
    //
    // Returns: void
    //
    // Remarks:
    //
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    public void moveBwd(double speed, double target) {

        double corr = 0.2;
        double angle = 0;
        double delta; // The difference between the target and measured angle

        angle = driveGyro.getAngle();

        delta = angle - target;

        // According to the documentation for DifferentialDrive.arcadeDrive(speed,turn)
        // the arguments are squared to accommodate lower drive speeds. If for example
        // the gain coefficient is 0.05 and the angle error is 5 degrees, the turning
        // argument would be 0.25*0.25 = 0.0625. This is a pretty slow correction.
        // We needed a larger correction factor - trying 0.2 for now. The range for
        // the turn is -1.0 to 1.0. Positive values are said to turn clockwise,
        // negatives counterclockwise.
        mecanumDrive.driveCartesian(0, -speed, -corr * delta);

        // System.out.println(" target = " + target + " angle = " + angle + " delta = "
        // + delta);

    }

    ///////////////////////////////////////////////////////////////////////////
    // Function: calcCounts_SAE(double distance)
    ///////////////////////////////////////////////////////////////////////////
    //
    // Purpose: Computes the encoder readout corresponding to the submitted
    // distance.
    //
    // Arguments: Accepts a double representing the distance in feet.
    //
    // Returns: A double representing the encoder change associated
    // with that distance.
    //
    // Remarks: ENCODER_RESOLUTION is the value associated with one
    // inch of travel. Based on eight inch diameter wheels used
    // on the 2019 robot.
    //
    // Example: If we are traveling 10 inches, the expected change
    // in output from the encoder would be:
    //
    // 10.0/1.537 = 6.506
    //
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    public double calcCounts_SAE(double distance) {
        double enc_change;

        distance *= 12.0; // convert feet to inches.

        enc_change = distance * ENCODER_RESOLUTION;

        return (enc_change);

    }

    // Given the counts, returns the distance in inches.
    public double calcDistance_SAE(double counts) {
        double distance;

        distance = counts * ENCODER_RESOLUTION;

        return (distance);

    }

    ///////////////////////////////////////////////////////////////////////////
    // Function: void calcCounts_Metric(double radius,double distance)
    ///////////////////////////////////////////////////////////////////////////
    //
    // Purpose: Given the distance to be traveled in meters, this function
    // calculates the encoder change required to travel that distance.
    //
    // Arguments: double distance (in meters).
    //
    // Returns: A double representing the encoder change for specified distance.
    //
    // Remarks:
    //
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    public double calcCounts_Metric(double distance) {
        double enc_change;

        distance *= 100.0; // convert meters to centimeters
        distance /= 2.54; // convert centimeters to inches

        enc_change = distance / ENCODER_RESOLUTION;

        return (enc_change);

    }

    /////////////////////////////////////////////////////////////////////
    // Function: public double turnRight_Arcade(double degrees)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Rotates the robot CW through the angle specified
    // by degrees.
    //
    // Returns: A double representing the error.
    //
    // Remarks: Uses Arcade drive to rotate the robot. Note that we
    // want motor braking enabled. Added escape count to exit
    // loop after a certain time when it doesn't reach the target.
    //
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public double turnRight_Arcade(double degrees) {
        int count = 0;
        double rot_speed = ROT_SPEED;
        double angle; // current gyro angle
        double target; // target angle
        double error;

        angle = driveGyro.getAngle(); // this is our starting point
        target = angle + degrees;
        System.out.println("ANGLE = " + angle + " Target = " + target);

        while (angle < target) {
            rotatePosArcade(rot_speed);
            angle = driveGyro.getAngle();

            if (angle > (target - ANGL_PROX_1)) {
                rot_speed /= ROT_ATTEN;
            }
            if (angle > (target - ANGL_PROX_2)) {
                rot_speed /= ROT_ATTEN;
            }
            delay.delay_seconds(0.01);
            count++;
            if ((count % GYRO_CONSOLE_UPDATE) == 0) {
                System.out.println("angle = " + angle);
            }
            if (count == GYRO_LOOP_ESCAPE)
                break;
        }
        mecanumDrive.driveCartesian(0, 0, 0);
        angle = driveGyro.getAngle();
        error = target - angle; // the error
        System.out.println("Turning Error = " + error + " degrees");
        return (error);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: public void rotatePosArcade(double rot_spd)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Uses arcadeDrive(...) to rotate the robot CW
    //
    // Arguments:Accepts a double representing the rotation speed.
    //
    // Returns: void
    //
    // Remarks: According to the documentation of this function the
    // argument "rot_speed" is squared.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public int rotatePosArcade(double rot_spd) {

        if (rot_spd > 1.0) {
            return (-1);
        }
        mecanumDrive.driveCartesian(0, 0, rot_spd);
        return (0);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: public double turnLeft_Arcade(double degrees)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Rotates the robot CCW through the specified number
    // of degrees.
    //
    // Arguments: The degrees of CCW rotation.
    //
    // Returns: A double representing the error.
    //
    // Remarks: Uses Arcade drive to
    // rotate the robot. Note that we want motor braking
    // enabled.
    //
    //
    // The final variable ROT_ATTEN value is yet to be determined but
    // for the first iteration it is 2.0.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public double turnLeft_Arcade(double degrees) {
        int count = 0;
        double rot_speed = ROT_SPEED;
        double angle; // current gyro angle
        double target;
        double error;

        // Current angle
        angle = driveGyro.getAngle();
        target = angle - degrees;

        while (angle > target) {
            rotateNegArcade(rot_speed);
            angle = driveGyro.getAngle();

            if (angle < (target + ANGL_PROX_1)) {
                rot_speed /= ROT_ATTEN;
            }
            if (angle < (target + ANGL_PROX_2)) {
                rot_speed /= ROT_ATTEN;
            }
            delay.delay_seconds(0.01);
            count++;
            if ((count % GYRO_CONSOLE_UPDATE) == 0) {
            }
            if (count == GYRO_LOOP_ESCAPE)
                break;
        }
        mecanumDrive.driveCartesian(0, 0, 0);
        angle = driveGyro.getAngle();
        System.out.println("angle = " + angle);
        error = target - angle;
        System.out.println("Turning Error = " + error + " degrees");
        return (error);

    }

    /////////////////////////////////////////////////////////////////////
    // Function: public int rotateNegArcade(double rot_spd)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Uses arcadeDrive(...) to rotate the robot CCW
    //
    // Arguments:Accepts a double representing the rotation speed.
    //
    // Returns: Zero normally, will return -1 if an unacceptable
    // rotation speed is entered
    //
    // Remarks: According to the documentation of this function the
    // argument "rot_speed" is squared.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public int rotateNegArcade(double rot_spd) {
        if (rot_spd > 1.0) {
            return (-1);
        }
        mecanumDrive.driveCartesian(0, 0, -rot_spd);
        return (0);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: public double turnAbsolute(double degrees)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Rotate the number of degrees in the direction specified
    // by the sign of the argument "degrees". Negative arguments will
    // rotate CCW, positive arguments CW.
    //
    // Arguments: A double representing the number of degrees to rotate
    //
    //
    // Returns: A double representing the difference between the
    // achieved rotation and the requested rotation.
    //
    // Remarks: A few test cases:
    //
    // 1. Initial angle measurement is 110 degrees, we request a -35
    // degree rotation. Target is then 100-35=65 degrees. We
    // rotate ccw to 65 degrees.
    // 2. Initial angle measurement is -45 and we ask for 360. New
    // target is 315. We rotate cw all the way around to 315.
    //
    // Everything depends on the functions turnRight/Left_Arcade(...).
    // The braking algorithms will determine the accuracy.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public double turnAbsolute(double degrees) {

        // Rotation Direction Flags
        boolean ccw = false;
        boolean cw = false;

        double result = 0.0;

        if (degrees < 0.0) { // we will rotate ccw
            ccw = true;
            cw = false;
        } else { // we rotate cw
            cw = true;
            ccw = false;
        }

        degrees = Math.abs(degrees);
        if (cw == true) {
            result = turnRight_Arcade(degrees);
        } else if (ccw == true) {
            result = turnLeft_Arcade(degrees);
        }

        return (result);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: public double turn2Heading(double heading)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Turns the robot to a compass heading (0->360).
    //
    // Arguments:Accepts a double representing the target heading
    //
    // Returns: The achieved heading or a ridiculous value in the
    // event that a negative heading is entered.
    //
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public double turn2Heading(double heading) {
        double angle;
        double delta;
        double change;
        double result;

        // Heading represents a "compass" reading, i.e.,
        // an angle from zero to something less than 360 degrees.
        // Negative arguments are not allowed. Because we want
        // to return the achieved heading we do not allow
        // submission of negative arguments and should return
        // a negative double so that the user can recognize
        // an error.
        if ((heading < 0.0) || (heading > 360.0)) {
            System.out.println("Submitted arguments must be greater than zero and less than 360");
            return (-999.9);
        }

        // This function will return an angle between 0 and 360 degrees.
        angle = getHeading();

        delta = heading - angle; // number of degrees to turn

        System.out.println("angle = " + angle + " delta = " + delta);

        // Reduce delta to the smallest necessary rotation
        // It should be something less than 180.0 or greater
        // than -180.0
        if (delta > 180.0) {
            delta -= 360.0;
        }
        if (delta < -180.0) {
            delta += 360.0;
        }

        System.out.println("delta = " + delta);

        // Depending on the sign of delta we turn left (-) or right (+)
        change = turnAbsolute(delta);
        System.out.println("delta = " + delta + " change = " + change);

        // Measure the angle, convert to compass indication. Note that
        // reading of the gyro after a small rotation could still be
        // outside the range of compass indications. We want to put
        // it in the range of the compass (0->360)
        result = getHeading();
        return (result);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: public double getHeading()
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Utility routine to determine starting compass indication
    // of the robot.
    //
    // Arguments:none
    //
    // Returns: The current heading of the robot, 0->360 degrees.
    //
    // Remarks: Called twice within turn2Heading.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public double getHeading() {
        int debug = 1;
        double angle;

        // A heading represents a "compass" reading, i.e.,
        // an angle from zero to something less than 360 degrees.

        // In the event that the current gyroscope indication is greater than
        // 360 degrees, reduce it to something within the compass range.
        // The same argument applies if the initial indication is less
        // than -360.0 degrees.
        angle = driveGyro.getAngle();
        while (angle > 360.0) {
            angle -= 360.0;
            if (debug == 1) {
                System.out.println("Compass heading = " + angle);
            }
        }

        // In the event that the current gyroscope indication is less
        // than -360.0, increase it to something within the compass range.
        while (angle < -360.0) {
            angle += 360.0;
            if (debug == 1) {
                System.out.println("Compass heading = " + angle);
            }
        }
        System.out.println("Compass heading = " + angle);

        return (angle);
    }



   /////////////////////////////////////////////////////////////////
    //  Function:  int fullStop()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Stops all drive motors
    //
    //  Arguments: void
    //  
    //  Returns:  Returns zero
    //
    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////
    int fullStop()
    {
        // Stop the motors, because we're at our target.
        frontLeftDriveMotor.set(0);
        backLeftDriveMotor.set(0);
        frontRightDriveMotor.set(0);
        backRightDriveMotor.set(0);

        return(0);

    
    }



 
}





