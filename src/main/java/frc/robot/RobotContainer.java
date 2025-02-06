// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.resetPigeon;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystem.SwerveSubsystem;
import swervelib.SwerveInputStream;

//TO-DO turn needs to be inverted
public class RobotContainer {
  private final SwerveSubsystem driveBase = new SwerveSubsystem();
  private final CommandXboxController driverController = new CommandXboxController(0);


  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
    driveBase.setDefaultCommand(driveFieldOrientatedAngularVelocity);

    NamedCommands.registerCommand("example", Commands.print("Hello World"));

    //to add auto, create auto in pathplanner
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(), 
                                            () -> driverController.getLeftY() * 1,
                                            () -> driverController.getLeftX() * 1)
                                            .withControllerRotationAxis(driverController::getRightX)
                                            .deadband(OperatorConstants.Deadzone)
                                            .scaleTranslation(1)
                                            .allianceRelativeControl(false);

   /*SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
                                          driverController::getRightX, 
                                          driverController::getRightY)
                                         .headingWhile(true);*/

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true);

    //Command driveFieldOrientatedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientatedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);
    Command driverobotOrientedCmd = driveBase.driveFieldOriented(driveRobotOriented);

  private void configureBindings() {
    driverController.y().onTrue(new resetPigeon(driveBase));
    driverController.x().whileTrue(driverobotOrientedCmd);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}