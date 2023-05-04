/////////////////////////////////////////////////////////////////////
// File: BallShooter.java
/////////////////////////////////////////////////////////////////////
//
// Purpose: Houses motors, functions, and other stuff for shooting
// power cells.
//
// Environment: Microsoft VSCode Java.
//
// Remarks: Created on 2/29/2020.
// First tried using inheritance in this file on 2/29/2020.
//
//  12/05/21:  Eliminated reference to WPI_TalonFX, replaced
//  with TalonFX.
//  Problems with SpeedControllerGroup - used separate control
//  of the motors.  Not sure what the consequences will be.
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
package frc.robot;


import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
//import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

class BallShooter extends Constants {

    // Creating the 4 shooter Falcon 500s, and assigning them their ID's.
    TalonFX frontLeftShooterMotor;
    TalonFX frontRightShooterMotor;
    TalonFX backLeftShooterMotor;
    TalonFX backRightShooterMotor;

    // Creating the SpeedControllerGroups linking the front and back
    // shooter motors together.
    SpeedControllerGroup frontShooterMotors;
    SpeedControllerGroup backShooterMotors;

    // Constructor.
    BallShooter() {

        frontLeftShooterMotor = new TalonFX(FRONT_LEFT_SHOOTER_MOTOR_ID);
        frontRightShooterMotor = new TalonFX(FRONT_RIGHT_SHOOTER_MOTOR_ID);
        backLeftShooterMotor = new TalonFX(BACK_LEFT_SHOOTER_MOTOR_ID);
        backRightShooterMotor = new TalonFX(BACK_RIGHT_SHOOTER_MOTOR_ID);

        //frontShooterMotors = new SpeedControllerGroup(frontLeftShooterMotor, frontRightShooterMotor);
        //backShooterMotors = new SpeedControllerGroup(backRightShooterMotor, backLeftShooterMotor);

        // Setting the shooter motors in coast mode.
        frontLeftShooterMotor.setNeutralMode(NeutralMode.Coast);
        frontRightShooterMotor.setNeutralMode(NeutralMode.Coast);
        backLeftShooterMotor.setNeutralMode(NeutralMode.Coast);
        backRightShooterMotor.setNeutralMode(NeutralMode.Coast);

        // Invert some of the motors, so they spin the right way.
        frontRightShooterMotor.setInverted(true);
        backRightShooterMotor.setInverted(true);
    }

    /////////////////////////////////////////////////////////////////////
    // Function: ballShoot()
    /////////////////////////////////////////////////////////////////////
    //
    // Purpose: Function used for shooting balls.
    // You can input what speeds you want the motors to run at when the
    // button is pressed.
    //
    // Arguments: double frontFalconSpeed, double backFalconSpeed
    // Speeds for the front and back motors, respectively.
    //
    // Returns: void
    //
    // Remarks: Created on 2/29/2020.
    //
    /////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////
    public void beltWheelMove(double backFalconSpeed) {

        //backShooterMotors.set(backFalconSpeed);
        backLeftShooterMotor.set(TalonFXControlMode.PercentOutput,backFalconSpeed);
        backRightShooterMotor.set(TalonFXControlMode.PercentOutput,backFalconSpeed);
    }

    public void ballWheelMove(double frontFalconSpeed) {
        //frontShooterMotors.set(frontFalconSpeed);
        frontLeftShooterMotor.set(TalonFXControlMode.PercentOutput,frontFalconSpeed);
        frontRightShooterMotor.set(TalonFXControlMode.PercentOutput,frontFalconSpeed);
    }

}

