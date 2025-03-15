// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final SparkMax elevatorMotor = new SparkMax(14, MotorType.kBrushless);
  public RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();



  public Elevator() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  //TO-DO Create PIDController to calculate speed of elevator going up
  public void elevatorSpeed(double elevatorSpeed){
    elevatorMotor.set(elevatorSpeed);
    SmartDashboard.putNumber("elevatorencoder", elevatorEncoder.getPosition() * -1);
  }


  public double ElevatorTravel(){
    double elevatorTicks = (elevatorEncoder.getPosition() * -1);

    return elevatorTicks;
  }




  //TO-DO Create Method to get encoder value reading from NEO Motor
  //TO-DO Get set encoder values for Intake, L1, L2, L3, L4, "Home"
  

}
