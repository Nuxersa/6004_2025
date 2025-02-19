package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;;

public class Elevator extends SubsystemBase {

    private final SparkMax elevatorMotor;
    private final SparkMax elevatorMotorFollow;
    
    /**
     * This subsytem that controls the arm.
     */
    public Elevator () {

    // Set up the arm motor as a brushed motor
    elevatorMotor = new SparkMax(ElevatorConstants.LIFT_MAIN, MotorType.kBrushless);
    elevatorMotorFollow = new SparkMax(ElevatorConstants.LIFT_MAIN, MotorType.kBrushless);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    elevatorMotor.setCANTimeout(250);
    elevatorMotorFollow.setCANTimeout(250);

    // Create and apply configuration for arm motor. Voltage compensation helps
    // the arm behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the arm stalls.
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorConfig.voltageCompensation(10);
    elevatorConfig.smartCurrentLimit(ElevatorConstants.LIFT_CUR_LMT);
    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorMotorFollow.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    public void moveElevator(double speed){
        elevatorMotor.set(speed);
        elevatorMotorFollow.set(-speed);
    }
}