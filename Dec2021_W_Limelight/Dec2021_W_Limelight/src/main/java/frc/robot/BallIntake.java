/////////////////////////////////////////////////////////////////////
// File: BallIntake.java
/////////////////////////////////////////////////////////////////////
//
// Purpose: Houses motors, functions, and other stuff for intaking
// power cells.
//
// Environment: Microsoft VSCode Java.
//
// Remarks: Created on 2/29/2020.
//
//  12/05/21:  Eliminated reference to WPI_TalonFX, replaced
//  with TalonFX.
//  Problems with SpeedControllerGroup - used separate control
//  of the motors.  Not sure what the consequences will be.
//  Changed the argument listed in the the function
//  intakeBalls( ... ) to be consistent with the required
//  variables.  Needed class reference to use motors from
//  BallShooter.
// 
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

class BallIntake extends Constants {

    // Creating the Falcon 500 for the ball intake, and assigning it an ID.
    TalonFX ballIntakeMotor;

    BallIntake() {

        ballIntakeMotor = new TalonFX(BALL_INTAKE_MOTOR_ID);

        // Setting the intake Falcon 500 motor in coast mode.
        ballIntakeMotor.setNeutralMode(NeutralMode.Coast);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: intakeBalls(...)
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Function used for intaking balls.
    // Runs the intake motor itself, and also the top belt of the shooter.
    //
    // Arguments: double intakeSpeed (speed of the intake motor),
    //            double topBeltSpeed (speed of the belt on the top of the robot).
    //
    // Returns: void
    //
    // Remarks: Created on 2/29/2020.
    //
    //  12/05/2021:  Added second argument "topBeltSpeed" to be 
    //  consistent with intended operation.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public void intakeBalls(double intakeSpeed,double topBeltSpeed) {

        // Run the intake motor at the inputted speed.
        ballIntakeMotor.set(TalonFXControlMode.PercentOutput,intakeSpeed);

        // Run the back shooter motors at the inputted speed.
        ballShooter.backLeftShooterMotor.set(TalonFXControlMode.PercentOutput,topBeltSpeed);
        ballShooter.backRightShooterMotor.set(TalonFXControlMode.PercentOutput,topBeltSpeed);
    }
}