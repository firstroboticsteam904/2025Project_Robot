// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;



public class intakeClaw extends SubsystemBase {
  /** Creates a new intakeClaw. */
  private final SparkMax brushMax = new SparkMax(15, MotorType.kBrushed);
  public static DoubleSolenoid intakeSolenoid = Robot.bingoPneumaticHub.makeDoubleSolenoid(7, 9);


  //TO-DOelea
  //Add Pneumatics, Add Intake/Output Motor
  //Get current Velocity of motor to know if game piece is grabbed or not
  public intakeClaw() {
    intakeSolenoid.set(Value.kReverse);
  }

  public void clawSpeed(double speed) {
    brushMax.set(speed);
  }

  public double clawMotorCurrent(){
    return SmartDashboard.getNumber("Claw Motor Current", brushMax.getOutputCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    clawMotorCurrent();
  }

  public void intakePivotOut(){
  intakeSolenoid.set(Value.kForward);
}

public void intakePrivotin(){
  intakeSolenoid.set(Value.kReverse);
}

}
