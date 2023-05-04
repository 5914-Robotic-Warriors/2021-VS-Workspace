// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  TalonFX motor1;
  TalonFX motor2;
  DoubleSolenoid d_solenoid;
  Joystick stick;
  Compressor comp;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    motor1 = new TalonFX(1);
    motor2 = new TalonFX(2);

    d_solenoid = new DoubleSolenoid(0, 1);
    comp = new Compressor();
    
    stick = new Joystick(0);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    motor1.set(ControlMode.PercentOutput, -stick.getRawAxis(1));
    motor2.set(ControlMode.PercentOutput, stick.getRawAxis(5));

    if(stick.getRawButton(2) == true) {
      comp.setClosedLoopControl(true);
    } else {
      comp.setClosedLoopControl(false);
    }

    if(stick.getRawButton(3) == true) {
      d_solenoid.set(DoubleSolenoid.Value.kForward);
    } else if (stick.getRawButton(3) == false) {
      d_solenoid.set(DoubleSolenoid.Value.kReverse);
    }

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
