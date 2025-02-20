// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class climber extends SubsystemBase {

  /** Creates a new climber. */
  public static DoubleSolenoid climberSolenoid = Robot.bingoPneumaticHub.makeDoubleSolenoid(0, 1);

  public climber() {
    climberSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberUpGraby() {
    climberSolenoid.set(Value.kForward);
  }

    public void climberDownGraby() {
       climberSolenoid.set(Value.kReverse);
    }

}
