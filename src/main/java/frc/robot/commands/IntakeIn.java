// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSub;
import edu.wpi.first.wpilibj2.command.Command;

/** An liftUpCommand that uses an lift subsystem. */
public class IntakeIn extends Command {
  private final IntakeSub m_intake;

  /**
   * Powers the lift up, when finished passively holds the lift up.
   * 
   * We recommend that you use this to only move the lift into the hardstop
   * and let the passive portion hold the lift up.
   *
   * @param lift The subsystem used by this command.
   */
  public IntakeIn(IntakeSub input) {
    m_intake = input;
    addRequirements(input);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.moveIntake(IntakeConstants.INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  // Here we run arm down at low speed to ensure it stays down
  // When the next command is caled it will override this command
  @Override
  public void end(boolean interrupted) {
    m_intake.moveIntake(IntakeConstants.INTAKE_SPEED_HOLD);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
