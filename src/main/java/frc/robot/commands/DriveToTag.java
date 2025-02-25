// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.Telemetry;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.PivotSub;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;
import frc.robot.util.LimelightHelpers;

/** An liftUpCommand that uses an lift subsystem. */
public class DriveToTag extends Command {
  //private final DriveToTag m_intake;
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /**
   * Powers the lift up, when finished passively holds the lift up.
   * 
   * We recommend that you use this to only move the lift into the hardstop
   * and let the passive portion hold the lift up.
   *
   * @param lift The subsystem used by this command.
   */
  /*public DriveToTag(DriveToTag input) {
    m_intake = input;
    addRequirements(input);
    
  }*/

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
        // Move the robot with corrected X velocity (compensating for the off-center camera)
        drivetrain.applyRequest(() -> drive.withVelocityX(-limelight_range_proportional())
        .withVelocityY(-.5 * MaxSpeed*(.4))
        .withRotationalRate(limelight_aim_proportional()));
    
  }

  // Called once the command ends or is interrupted.
  // Here we run arm down at low speed to ensure it stays down
  // When the next command is caled it will override this command
  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> 
        // Move the robot with corrected X velocity (compensating for the off-center camera)
        drive.withVelocityX(0) // Only use range for X (forward/backward)
            .withVelocityY(0)  // We won't use Y for correction, just driving forward
            .withRotationalRate(0)  // Add offset correction to rotation
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Simple proportional turning control with Limelight.
double limelight_aim_proportional() {    
  double kP = 0.175; // Control constant for turning aggressiveness

  // Get the horizontal offset (tx) from the Limelight
  double targetingAngularVelocity = LimelightHelpers.getTY("limelight") * kP;

  // Scale the angular velocity to MaxAngularRate for controlling robot's rotation speed
  targetingAngularVelocity *= MaxAngularRate;

  // Invert the angular velocity since tx is positive when the target is to the right of the crosshair
  targetingAngularVelocity *= -1.0;

  return targetingAngularVelocity;
}

// Simple proportional control for range based on ty (vertical angle)
double limelight_range_proportional() {    
  double kP = 0.1;  // Proportional control constant for range
  double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
  
  // Adjust forward speed proportionally based on target distance
  targetingForwardSpeed *= MaxSpeed;
  
  // Invert the forward speed to move toward the target (negative ty means farther away)
  targetingForwardSpeed *= -1.0;

  return targetingForwardSpeed;
}

// Compensation for the Limelight's 11.5-inch offset from the robot's center
double limelight_x_offset_correction() {
  double limelight_offset_in_inches = 11.5;  // Limelight offset in inches
  double limelight_offset_scale = limelight_offset_in_inches / 12.0; // Convert to feet for scaling
  double x_offset_correction = limelight_offset_scale * MaxSpeed; // Apply scaling to the robot's MaxSpeed

  // This offset helps adjust the X direction to compensate for the off-center camera
  return x_offset_correction;
}
}
