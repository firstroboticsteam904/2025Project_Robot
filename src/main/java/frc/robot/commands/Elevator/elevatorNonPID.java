// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevatorNonPID extends Command {
  /** Creates a new elevatorNonPID. */
  Elevator elevator;
  double MAXRangeElevatorTicks;
  double MINRangeElevatorTicks;
  double emergencyUpTicks = 127;
  double approachingEmergencyTicks = 120;
  public elevatorNonPID(Elevator elevator, double MAXelevatorTicks, double MINelevatorTicks) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    MAXRangeElevatorTicks = MAXelevatorTicks;
    MINRangeElevatorTicks = MINelevatorTicks;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double deathtrapTravel = elevator.ElevatorTravel();
    if(deathtrapTravel < MAXRangeElevatorTicks){
      elevator.elevatorSpeed(-0.4);
    } else if(deathtrapTravel > MINRangeElevatorTicks){
      elevator.elevatorSpeed(0.3);
    } else if(deathtrapTravel >= emergencyUpTicks){
      RobotContainer.operaterController.setRumble(RumbleType.kBothRumble, 1);
      elevator.elevatorSpeed(0);
    } else if(deathtrapTravel >= approachingEmergencyTicks){
      RobotContainer.operaterController.setRumble(RumbleType.kBothRumble, 0.5);
      elevator.elevatorSpeed(-0.2);
    } else{
      RobotContainer.operaterController.setRumble(RumbleType.kBothRumble, 0);
      elevator.elevatorSpeed(0.025);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(elevator.ElevatorTravel() <= MAXRangeElevatorTicks && elevator.ElevatorTravel() >= MINRangeElevatorTicks){
      elevator.elevatorSpeed(-0.025);
      return true;
    } else if(elevator.ElevatorTravel() >= emergencyUpTicks){
      elevator.elevatorSpeed(0);
      return true;
    }
    else {
      return false;
    }
  }
}
