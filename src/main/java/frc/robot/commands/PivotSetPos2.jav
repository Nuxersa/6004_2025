package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.Constants.PivotConstants;




    public class PivotSetPos2 extends CommandBase {
        private final PivotSubsystem pivotSubsystem;
        private final double position =  PivotConstants.PIVOT_EXTEND_1;
        public PivotSetPos2(PivotSubsystem pivotSubsystem, double position) {
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

