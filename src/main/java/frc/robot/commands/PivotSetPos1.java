package frc.robot.commands;

import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.PivotSub;
import edu.wpi.first.wpilibj2.command.Command;


    public class PivotSetPos1 extends Command {
        private final PivotSub pivotSubsystem;
        private final double position =  PivotConstants.PIVOT_EXTEND_0;
        public PivotSetPos1(PivotSub pivotSubsystem, double position) {
            this.pivotSubsystem = pivotSubsystem;
            this.position = position;
            addRequirements(pivotSubsystem);
        }

        @Override
        public void initialize() {
            pivotSubsystem.setPosition(position);
        }

        @Override
        public void execute() {
            // Optionally add code to monitor or adjust during execution
        }

        @Override
        public boolean isFinished() {
            return pivotSubsystem.isAtPosition(position);
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                pivotSubsystem.stop();
            }
        }
    }

