// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevatorUp extends Command {
  /** Creates a new elevatorMove. */
  Elevator elevator;
  double desiredElevatorTicks;
  double emergencyTicks = 130;
  public elevatorUp(Elevator elevator, double elevatorTicks) {
    this.elevator = elevator;
    desiredElevatorTicks = elevatorTicks;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double elevatorGo = elevator.ElevatorTravel();
      if(elevatorGo <= desiredElevatorTicks){
        elevator.elevatorSpeed(-0.5);
      } else if(elevatorGo >= emergencyTicks){
        elevator.elevatorSpeed(0);
      } else{
        elevator.elevatorSpeed(0);
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (elevator.ElevatorTravel() >= desiredElevatorTicks || elevator.ElevatorTravel() >= emergencyTicks) {
      elevator.elevatorSpeed(0);
      return true;
    }
    else{
      return false;
    }
  }
}
