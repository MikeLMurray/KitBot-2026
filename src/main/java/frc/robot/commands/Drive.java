package frc.robot.commands;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CANDriveSubsystem;

public class Drive extends Command {
  private final CANDriveSubsystem driveSubsystem;
  private final CommandXboxController controller;

  public Drive(CANDriveSubsystem driveSystem, CommandXboxController driverController) {
    addRequirements(driveSystem);

    driveSubsystem = driveSystem;
    controller = driverController;

    SmartDashboard.putNumber("Driver Left Y", 0.0);
    SmartDashboard.putNumber("Forward Cmd", 0.0);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Drive Command Running", true);
  }

  @Override
  public void execute() {
    double leftY = controller.getLeftY();
    double rightX = controller.getRightX();

    double forward = -leftY * DRIVE_SCALING;
    double turn = -rightX * ROTATION_SCALING;

    // Raw controller values
    SmartDashboard.putNumber("Driver Left Y", leftY);
    SmartDashboard.putNumber("Driver Right X", rightX);

    // Scaled drive commands
    SmartDashboard.putNumber("Forward Cmd", forward);
    SmartDashboard.putNumber("Turn Cmd", turn);

    // Command status
    SmartDashboard.putBoolean("Drive Command Running", true);

    driveSubsystem.driveArcade(forward, turn);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Drive Command Running", false);
    SmartDashboard.putNumber("Driver Left Y", 0.0);
    SmartDashboard.putNumber("Forward Cmd", 0.0);
    driveSubsystem.driveArcade(0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}