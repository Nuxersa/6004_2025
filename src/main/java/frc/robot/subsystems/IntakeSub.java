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

public class IntakeSub extends SubsystemBase {
    TalonFX intakeFront;
    TalonFX intakeBack;
    
    
    /**
     * This subsytem that controls the arm.
     */
    public IntakeSub () {
        intakeFront = new TalonFX(IntakeConstants.INTAKE_FRONT_ID);
        intakeBack = new TalonFX(IntakeConstants.INTAKE_BACK_ID);
        
        var intakeFrontConfiguration = new TalonFXConfiguration();
        var intakeBackConfiguration = new TalonFXConfiguration();

        intakeFrontConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeFrontConfiguration.CurrentLimits.SupplyCurrentLimit = 55;
        intakeFrontConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeBackConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeBackConfiguration.CurrentLimits.SupplyCurrentLimit = 55;
        intakeBackConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeFront.setControl(new Follower(intakeBack.getDeviceID(), true));
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
    public void moveIntake(double speed){
        intakeFront.set(speed);
        intakeBack.set(speed);
    }
}