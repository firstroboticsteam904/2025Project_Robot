// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCMDs;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Intake.*;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.intakeClaw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Level3CMD extends ParallelCommandGroup {
  /** Creates a new IntakeLevelCMD. */
  Elevator kElevator;
  intakeClaw kIntakeClaw;
  public Level3CMD(Elevator kElevator, intakeClaw kIntakeClaw) {
    this.kElevator = kElevator;
    this.kIntakeClaw = kIntakeClaw;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new elevatorNonPID(kElevator, 72, 69),
      new  intakesolenoidIN(kIntakeClaw)
    );
  }
}
