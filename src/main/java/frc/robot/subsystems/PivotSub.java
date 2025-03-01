package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//custom
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.TalonFXConstants;



public class PivotSub extends SubsystemBase {
    TalonFX intakePivot;
    TalonFX intakeGrab;
    
    
    /**
     * This subsytem that controls the arm.
     */
    public PivotSub () {
        intakePivot = new TalonFX(IntakeConstants.INTAKE_Pivot_ID);
        
        var intakePivotConfiguration = new TalonFXConfiguration();

        intakePivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakePivotConfiguration.CurrentLimits.SupplyCurrentLimit = 55;
        intakePivotConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true; 
        // in init function, set slot 0 gains
        var slot0Configs = new TalonFXConfiguration.Slot0Configs();
        slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        intakePivot.getConfigurator().apply(slot0Configs);

        //getConfigurator().apply(slot0Configs);

        intakePivot.getConfigurator().apply(intakePivotConfiguration);
        // Set the initial position to zero
        intakePivot.setSelectedSensorPosition(0);

        // Method to set the desired position of the pivot arm
        //public void setPivotPosition(double position) {
        //    intakePivot.set(ControlMode.Position, position);
        //}
    }

    @Override
    public void periodic() {
    }
    /** 
     * This is a method that makes the arm move at your desired speed
     *  Positive values make it spin forward and negative values spin it in reverse
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void movePivot(double speed){
        intakePivot.set(speed);
    }
}