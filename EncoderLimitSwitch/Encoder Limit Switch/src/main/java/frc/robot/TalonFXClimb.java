/////////////////////////////////////////////////////////////////////
//   File:  TalonFXClimb.java
/////////////////////////////////////////////////////////////////////
//
//  Purpose:  Test routines in support of the climbing operation.
//
//  Programmer:
//
//  Compiler:  Microsoft VS for FRC, Java language
//
//  Revision:
//
//  Remarks:  These routines extend or retract the climbing arm.
//            The problem is how to stop the motor when one 
//            reaches the maximum extension or minimum retraction.
//            To do this entirely in software requires:
//
//            1.  An absolute reference point to determine
//                where the arm is relative to the maximum
//                and minimum extension/retraction.
//            2.  Control of the motor velocity as one approaches
//                the limits.
//
//            A better approach is to tie in a microswitch at the
//            limits that will kill motor power.  Need to investigate
//            if the TalonFX motor controller has an input for such
//            a device.
//
//            You will need to specify the extend distance or
//            retract distance in inches relative to your starting
//            position (at power up).
//
//            3/12/2021:  Starting position is equal to the 
//            retract position, i.e., zero.  Based the retract 
//            function on the error between full extension, 
//            position, and zero full retract position.
//
//
/////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////// 

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

public class TalonFXClimb {

    private double starting_output = 0.00; // zero the motor power

    // Fixed parameters associated with the climb function
    private final double worm_pitch = 5.0; // movement per turn in mm
    private final double gear_reduction = 4.0; // gear reduction motor shaft to worm shaft
    private final double counts_per_rev = 2048; // Characteristic of the encoder.

    TalonFX _talon;
    Delay delay;

    double motor_output;
    double max_output = 1.0; // You could play with this..

    // You will need to provide these dimensions.
    double upper_limit = 17.7; // distance in inches from reference point
    double lower_limit = 0.0; // distance in inches from reference point

    int loop_count = 0; // Used to limit the number of printouts for debugging

    double position;
    double extend;
    double retract;

    int retract_init=1;

    // count limits for extend/retract based on zero reference at power-up.
    double extend_target;
    double retract_target;

    //  difference between extend and retract limits.
    double spread;

    // Base constructor
    TalonFXClimb(int can_addr) {

        /* Hardware */
        _talon = new TalonFX(can_addr);
        delay = new Delay();

        /* Factory Default all hardware to prevent unexpected behaviour */
        // _talon.configFactoryDefault();
        System.out.println("Error Code = " + _talon.configFactoryDefault());

        /* Config sensor used for Primary PID [Velocity] */
        // Integrated Sensor (TalonFX has it), primary closed loop, 50 msec timeout
        _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);

        // configNominalOutputForward(double percentOutput,int timeoutMS)
        // These calls indicate 0 output nominal
        _talon.configNominalOutputForward(0, 50);
        _talon.configNominalOutputReverse(0, 50);

        // configPeakOutputForward(double percentOut,int timeoutMS)
        // These calls indicate maximum output
        // note sign reversal for reverse
        _talon.configPeakOutputForward(1, 50);
        _talon.configPeakOutputReverse(-1, 50);

        motor_output = starting_output;

        // Zero the encoder. It assumed that this constructor
        // will be called once at power up in robotinit()
        _talon.setSelectedSensorPosition(0.0);

        delay.delay_milliseconds(50); // need some delay here
        position = _talon.getSelectedSensorPosition(); // should be zero

        //  These are the encoder counts from the starting position (zero)
        //  to the fully extended and fully retracted positions
        extend = computeUpperCount();
        retract = computeLowerCount();
        spread=extend-retract;

        // Compute maximum extension/retraction targets
        extend_target = extend;
        retract_target = retract; // zero by definition, but could 
                                  //  represent some distance from zero.

        System.out.println("extend = " + extend);
        System.out.println("retract = " + retract);
        System.out.println("position = " + position);
        System.out.println("extend_target = " + extend_target);
        System.out.println("retract_target = " + retract_target);

    }

    /////////////////////////////////////////////////////////////////
    // Function: int computeUpperCount()
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Computes the count at full extension. Full
    // extension is "upper_limit" in inches.
    //
    // Arguments:void
    //
    // Remarks: This assumes we know our 'zero' reference.
    //
    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////
    double computeUpperCount() {
        double upper_count;
        double inches_per_rev; // The inches of travel per motor revolution
        double pitch; // pitch in inches

        pitch = worm_pitch / 25.4;

        inches_per_rev = pitch / gear_reduction; // account for gear reduction

        upper_count = (upper_limit / inches_per_rev) * counts_per_rev;

        return (upper_count);

    }

   /////////////////////////////////////////////////////////////////
    // Function: int computeLowerCount()
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Computes the count at lower limit. Full
    // retraction is "lower_limit" in inches.
    //
    // Arguments:void
    //
    // Remarks: This assumes we know our 'zero' reference.  the
    //          value of lower_limit is actually zero but to
    //          adjust for robot nuances can be set to something
    //          above zero.
    //
    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////
    double computeLowerCount() {
        double lower_count;
        double inches_per_rev; // The inches of travel per motor revolution
        double pitch; // pitch in inches

        pitch = worm_pitch / 25.4;

        inches_per_rev = pitch / gear_reduction; // account for gear reduction

        lower_count = (lower_limit / inches_per_rev) * counts_per_rev;

        return (lower_count);

    }

    ///////////////////////////////////////////////////////////////////
    // Function: int extend()
    //////////////////////////////////////////////////////////////////
    //
    // Purpose: Extends the arm to "upper_limit"
    //
    // Arguments:void
    //
    // Returns: Zero
    //
    // Remarks: 3/6/2021: Working. Note that the error results
    // in the motor "coasting" after being commanded to
    // zero. Error should reduce if braking mode is
    // applied and/or the loading of the arm. This
    // was tested with a free-running motor.
    //
    // This function is called over and over within
    // teleop().
    //
    //  Tested on robot 3/11/2021 and it works as advertised.
    //
    //
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    public int extend() {
        int debug = 1;

        // Temporary variables, calculated within this function
        double position;
        double error;

        // Get current postion. Note that this is relative
        // to the starting position of zero at power up.
        position = _talon.getSelectedSensorPosition();

        error = extend_target - position;

        if (error > 0.25 * extend) {
            motor_output = max_output;
            _talon.set(TalonFXControlMode.PercentOutput, motor_output);

        } else if ((error < 0.25 * extend) && (error > 0.10 * extend)) {
            motor_output = 0.5;
            _talon.set(TalonFXControlMode.PercentOutput, motor_output);

        } else if ((error < 0.10 * extend) && (error > 0.05 * extend)) {
            motor_output = 0.3;
            _talon.set(TalonFXControlMode.PercentOutput, motor_output);

        } else if (error < 0.0) { // overshoot
            motor_output = 0.0;
            _talon.set(TalonFXControlMode.PercentOutput, motor_output);
        }

        if (debug == 1) {
            loop_count++;

            if (loop_count == 20) {
                System.out.println("Target = " + extend_target);
                System.out.println("Position = " + position + "counts");
                System.out.println("Error = " + error);
                System.out.println("Motor Output = " + motor_output);
                loop_count = 0;
            }
        }

        return (0);

    }

    ///////////////////////////////////////////////////////////////////
    // Function: int retract()
    //////////////////////////////////////////////////////////////////
    //
    // Purpose: Retracts the arm to "lower_limit"
    //
    // Arguments:void
    //
    // Returns: Zero
    //
    // Remarks: 3/8/2021: Working. Note that the error results
    // in the motor "coasting" after being commanded to
    // zero. Error should reduce if braking mode is
    // applied and/or the loading of the arm. This
    // was tested with a free-running motor.
    //
    // This function is called over and over within
    // teleop().
    //
    // 03/12/2021:  Based on the actual robot, full retract
    // is to the zero position.  However, one might want to
    // set the full retract position to something above
    // zero.  This will allow abrupt motor shutoff when the
    // the error becomes less than zero.  The variable retract,
    //  e.g., retract_target determines the deadband.
    //  Error/power block is changed to be based on 
    //  full extension vs. position.
    //  Introduce fixed paramter "spread" , the difference in
    //  counts between full extensiona and full retraction
    //  to reduce computations within the error vs. motor 
    //  power if/else block.
    //
    //
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    public int retract() {
        int debug = 1;
        

        // Temporary variables, calculated within this function
        double position;
        double error;

        //  For testing purposes without a robot present.set the
        //  position somewhere between max and min extension.
        //  For normal operation of robot, comment this block
        //  out.
        // if(retract_init==1)  {
        //     _talon.setSelectedSensorPosition(500000.0);
        //     delay.delay_milliseconds(50);
        //     retract_init=0;
        // }

        // Get current postion. Note that this is relative
        // to the starting position of zero at power up.
        position = _talon.getSelectedSensorPosition();

        // Retract_target is set slightly above zero. Position will 
        // always be positive, error will always be positive.  
        // When error turns negative motor is shut off.  
        // If this function is called at the full retract position, 
        // motor command is zero as it should be.
        // Error will be positive until position crosses past the
        // lower_limit.
        error = position-retract_target; 


        //  Base error tree on the full extension value relative
        //  to lower_limit to position.

        if (error > 0.15 * spread) {
            motor_output = -max_output; // reverse direction
            _talon.set(TalonFXControlMode.PercentOutput, motor_output);

        } else if ((error < 0.15 * spread) && (error > 0.02 * spread)) {
            motor_output = -0.3;
            _talon.set(TalonFXControlMode.PercentOutput, motor_output);

        } else if ((error < 0.02 * spread) && (error > 0.01 * spread)) {
            motor_output = -0.1;
            _talon.set(TalonFXControlMode.PercentOutput, motor_output);

        } else if (error < 0.0) { // overshoot
            motor_output = 0.0;
            _talon.set(TalonFXControlMode.PercentOutput, motor_output);
        }

        if (debug == 1) {
            loop_count++;

            if (loop_count == 20) {
                System.out.println("Target = " + retract_target);
                System.out.println("Position = " + position + "counts");
                System.out.println("Error = " + error);
                System.out.println("Motor Output = " + motor_output);
                loop_count = 0;
            }
        }

        return (0);

    }

    /////////////////////////////////////////////////////////////////
    // Function:
    /////////////////////////////////////////////////////////////////
    //
    // Purpose:
    //
    // Arguments:
    //
    // Remarks:
    //
    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////
    // Function:
    /////////////////////////////////////////////////////////////////
    //
    // Purpose:
    //
    // Arguments:
    //
    // Remarks:
    //
    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////

}
