// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

import java.security.Timestamp;

import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/** An liftUpCommand that uses an lift subsystem. */
public class ElevatorJiggleCommand extends Command {
  private final Elevator m_elevator;

  /**
   * Powers the lift up, when finished passively holds the lift up.
   * 
   * We recommend that you use this to only move the lift into the hardstop
   * and let the passive portion hold the lift up.
   *
   * @param lift The subsystem used by this command.
   */
  public ElevatorJiggleCommand(Elevator lift) {
    m_elevator = lift;
    addRequirements(lift);
  }

    private final CommandXboxController op = new CommandXboxController(1);


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.moveElevator((Math.sin(System.currentTimeMillis() / 1)) / 2);
  }

  // Called once the command ends or is interrupted.
  // Here we run arm down at low speed to ensure it stays down
  // When the next command is caled it will override this command
  @Override
  public void end(boolean interrupted) {
    m_elevator.moveElevator(ElevatorConstants.LIFT_HOLD_DOWN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
