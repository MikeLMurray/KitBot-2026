// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  // SPARK MAX simulation models to compute applied output, velocity, and current for the Sim GUI
  private final SparkMaxSim leftLeaderSim;
  private final SparkMaxSim leftFollowerSim;
  private final SparkMaxSim rightLeaderSim;
  private final SparkMaxSim rightFollowerSim;

  private final DifferentialDrive drive;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    // create SPARK MAX simulation models
    leftLeaderSim = new SparkMaxSim(leftLeader, DCMotor.getCIM(2));
    leftFollowerSim = new SparkMaxSim(leftFollower, DCMotor.getCIM(2));
    rightLeaderSim = new SparkMaxSim(rightLeader, DCMotor.getCIM(2));
    rightFollowerSim = new SparkMaxSim(rightFollower, DCMotor.getCIM(2));

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Publish the virtual field to SmartDashboard so it appears in the Sim GUI / Glass
    SmartDashboard.putData("Field", field);

    // Initialize SmartDashboard telemetry
    SmartDashboard.putNumber("Left Motor Output", 0.0);
    SmartDashboard.putNumber("Left Velocity MPS", 0.0);

    // Initialize simulated battery voltage to nominal 12V
    if (RobotBase.isSimulation()) {
      RoboRioSim.setVInVoltage(12.0);
    }
  }

  // Drivetrain physics simulation model for standard KitBot (2 CIMs/side, 10.71:1 gearing, 6" wheels)
  private final DifferentialDrivetrainSim driveSim =
      DifferentialDrivetrainSim.createKitbotSim(
          DifferentialDrivetrainSim.KitbotMotor.kDualCIMPerSide,
          DifferentialDrivetrainSim.KitbotGearing.k10p71,
          DifferentialDrivetrainSim.KitbotWheelSize.kSixInch,
          null
      );

  // 2D Field widget to visualize the robot on the virtual field
  private final Field2d field = new Field2d();

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Motor Output", leftLeader.get());
    SmartDashboard.putNumber("Left Velocity MPS", getLeftVelocityMPS());
  }

  @Override
  public void simulationPeriodic() {
    double batteryVoltage = RobotController.getBatteryVoltage();

    // 1. Pass commanded motor voltages to the drivetrain simulation
    driveSim.setInputs(
        leftLeader.get() * batteryVoltage,
        rightLeader.get() * batteryVoltage
    );

    // 2. Advance the simulation by the standard robot loop period (20 ms)
    driveSim.update(0.02);

    // 3. Update battery voltage based on drivetrain current draw
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(driveSim.getCurrentDrawAmps())
    );

    // 4. Update REV SPARK MAX simulated devices so Applied Output, Velocity, etc. show up in the Sim GUI
    double wheelCircumferenceMeters = Units.inchesToMeters(6.0) * Math.PI;
    double gearRatio = 10.71;
    double leftMotorRPM = (driveSim.getLeftVelocityMetersPerSecond() / wheelCircumferenceMeters) * 60.0 * gearRatio;
    double rightMotorRPM = (driveSim.getRightVelocityMetersPerSecond() / wheelCircumferenceMeters) * 60.0 * gearRatio;

    leftLeaderSim.iterate(leftMotorRPM, batteryVoltage, 0.02);
    rightLeaderSim.iterate(rightMotorRPM, batteryVoltage, 0.02);
    leftFollowerSim.setAppliedOutput(leftLeaderSim.getAppliedOutput());
    rightFollowerSim.setAppliedOutput(rightLeaderSim.getAppliedOutput());

    // 5. Update the Field2d display with the simulated pose
    //field.setRobotPose(driveSim.getPose());
    // 5. Update the Field2d display with the corrected simulated pose
    Pose2d rawPose = driveSim.getPose();
    Pose2d correctedPose = new Pose2d(
    -rawPose.getX(), // Invert X to flip forward/backward visual direction
    rawPose.getY(), 
    rawPose.getRotation().times(-1) // Invert rotation to match the flip
);
field.setRobotPose(correctedPose);

  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  /** Returns the left wheel velocity in meters per second. */
  public double getLeftVelocityMPS() {
    if (RobotBase.isSimulation()) {
      return driveSim.getLeftVelocityMetersPerSecond();
    }
    return 0.0;
  }

  /** Returns the right wheel velocity in meters per second. */
  public double getRightVelocityMPS() {
    if (RobotBase.isSimulation()) {
      return driveSim.getRightVelocityMetersPerSecond();
    }
    return 0.0;
  }

  /** Returns the current simulated robot pose. */
  public Pose2d getPose() {
    return driveSim.getPose();
  }

  /** Resets the simulated robot pose to the specified position on the field. */
  public void resetPose(Pose2d pose) {
    driveSim.setPose(pose);
    field.setRobotPose(pose);
  }

  /** Returns the Field2d instance. */
  public Field2d getField() {
    return field;
  }

}
