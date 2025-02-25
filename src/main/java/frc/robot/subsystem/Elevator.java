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
  //TO-DO
  //Add elevator NEO motor
  public Elevator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void elevatorUp(){
    elevatorMotor.set(-0.5);
    System.out.println("Elevator going up!");
  }

  public void elevatorDown(){
    elevatorMotor.set(0.5);
    System.out.println("Elevator going down!");
  }

}
