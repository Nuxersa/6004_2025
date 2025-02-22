package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfigurator;
//custom
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.TalonFXConstants;

import com.ctre.phoenix6.hardware.CANrange;

public class GrabSub extends SubsystemBase {
    TalonFX intakeGrab;
 // Constants used in CANrange construction
 final int kCANrangeId = 0;
 
 // Construct the CANrange
 CANrange CANrange;
    
    /**
     * This subsytem that controls the arm.
     */
    public GrabSub () {
        intakeGrab = new TalonFX(IntakeConstants.INTAKE_Grab_ID);
        
        var intakeGrabConfiguration = new TalonFXConfiguration();

        intakeGrabConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeGrabConfiguration.CurrentLimits.SupplyCurrentLimit = 55;
        intakeGrabConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

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
    public void moveGrab(double speed){
        intakeGrab.set(speed);
        CANrange = new CANrange(kCANrangeId);
        
        // Configure the CANrange for basic use
        CANrangeConfiguration configs = new CANrangeConfiguration();
        
        // Write these configs to the CANrange
        CANrangeConfigurator configurator = CANrange.getConfigurator();
        configurator.apply(configs);
        
        // Get Distance
        var distance = CANrange.getDistance();
        
        // Refresh and print these values
        System.out.println("Distance is " + distance.refresh().toString());
    }
}