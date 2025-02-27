// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.util.LimelightHelpers;

//Team Subsystems
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PivotSub;
import frc.robot.subsystems.GrabSub;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorJiggleCommand;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.PivotIn;
import frc.robot.commands.PivotOut;
import frc.robot.commands.GrabIn;
import frc.robot.commands.GrabOut;
import frc.robot.commands.ElevatorSetPos1;
import frc.robot.commands.ElevatorSetPos2;
import frc.robot.commands.ElevatorSetPos3;
import frc.robot.commands.ElevatorSetPos4;
import frc.robot.commands.ElevatorSetPos5;
import frc.robot.commands.ReefAlignCommand;
import frc.robot.subsystems.vision.apriltag.AprilTagPose;
import frc.robot.subsystems.vision.apriltag.impl.limelight.LimelightAprilTagSystem;

import frc.robot.commands.DriveToTag;


public class RobotContainer {
    public final Elevator elevatorSubsystem = new Elevator();
    public final PivotSub pivotSubsystem = new PivotSub();
    public final GrabSub grabSubsystem = new GrabSub();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    @Logged(name = "Vision/Limelight")
    public final LimelightAprilTagSystem reefCamera;
    public ReefAlignCommand alignToReef;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //driver
    private final CommandXboxController joystick = new CommandXboxController(0);
    //op
    private final CommandXboxController op = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
         //commands for use in auto
        // Register Named Commands
        //NamedCommands.registerCommand("printme", Commands.print("This is command"));
        //new EventTrigger("exampleevent").onTrue(Commands.print("Passed an event marker"));
        //smart dashboard for Auto paths
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putData(elevatorSubsystem);
        SmartDashboard.putData(pivotSubsystem);
        SmartDashboard.putData(grabSubsystem);
        reefCamera = new LimelightAprilTagSystem("limelight", drivetrain);
        alignToReef =
                new ReefAlignCommand(
                        drivetrain,
                        reefCamera,
                        joystick::getLeftY,
                        joystick::getLeftX);
        configureBindings();
    }

    private final SendableChooser<Command> autoChooser;

    // Slew rate limiters to make driveStick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_elev = new SlewRateLimiter(4);

    public void updateVision() {
        Optional<AprilTagPose> aprilTagPoseOpt = reefCamera.getEstimatedPose();

        if (aprilTagPoseOpt.isPresent()) {
            AprilTagPose pose = aprilTagPoseOpt.get();

            if (pose.getNumTags() > 0) {
                drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                drivetrain.addVisionMeasurement(pose.getEstimatedRobotPose(), pose.getTimestamp());
            }
        }
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically

            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_xspeedLimiter.calculate(joystick.getLeftY()) * MaxSpeed * .6) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_yspeedLimiter.calculate(joystick.getLeftX()) * MaxSpeed * .6) // Drive left with negative X (left)
                    .withRotationalRate(-m_rotLimiter.calculate(joystick.getRightX()) * MaxAngularRate * .6) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //drive to tag
        joystick.rightBumper().whileTrue(
                //lime light aim any april tag                
                drivetrain.applyRequest(() -> drive.withVelocityX(-limelight_range_proportional())
                .withVelocityY(-.5 * MaxSpeed*(.4))
                .withRotationalRate(limelight_aim_proportional()))
        );
        
        //joystick.rightBumper().whileTrue(new DriveToTag());
        joystick.leftTrigger().whileTrue(alignToReef);

        // move pivot
        op.povDown().whileTrue(new PivotIn(pivotSubsystem));
        op.povUp().whileTrue(new PivotOut(pivotSubsystem));


        //move Grab!
        op.x().whileTrue(new GrabIn(grabSubsystem));
        op.y().whileTrue(new GrabOut(grabSubsystem));
    


        // move elevator
        op.leftTrigger(.05).whileTrue(new ElevatorUpCommand(elevatorSubsystem));
        op.rightTrigger(.05).whileTrue(new ElevatorDownCommand(elevatorSubsystem));


        //PID elevator testing
        joystick.povDown().whileTrue(new ElevatorSetPos1(elevatorSubsystem));
        joystick.povUp().whileTrue(new ElevatorSetPos4(elevatorSubsystem));


        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

// function to drive towards the AprilTag, considering the Limelight's offset
void driveToTag() {
    drivetrain.applyRequest(() -> drive.withVelocityX(-limelight_range_proportional())
        .withVelocityY(-.5 * MaxSpeed*(.4))
        .withRotationalRate(limelight_aim_proportional()));
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
