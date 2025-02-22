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