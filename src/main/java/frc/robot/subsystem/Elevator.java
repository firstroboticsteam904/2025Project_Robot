// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final SparkMax elevatorMotor = new SparkMax(13, MotorType.kBrushless);

  public Elevator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //TO-DO Create PIDController to calculate speed of elevator going up
  public void elevatorUp(){
    elevatorMotor.set(-0.5);

  }

  //TO-DO Create PIDController to calculate speed of elevator going down
  public void elevatorDown(){
    elevatorMotor.set(0.5);

  }

  //TO-DO Create Method to get encoder value reading from NEO Motor
  //TO-DO Get set encoder values for Intake, L1, L2, L3, L4, "Home"
  

}
