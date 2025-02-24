//NEEDS TUNING!!!!!
//^^^^^^^^^^^^^^^\\
//^^^^^^^^^^^^^^^\\
package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final SparkMax motor;          
    private final RelativeEncoder encoder;  
    private final PIDController pid;        
    
    // You'll need to adjust these based on your elevator's specifications
    private final double COUNTS_PER_INCH = 42.0; // Needs to be measured, not publically available
    private final double GRAVITY_COMPENSATION = (0.5 * 0.05); // edit if needed, should be good
    
    public Elevator() { // Replace SparkMax CANID
        motor = new SparkMax(0, MotorType.kBrushless);
        encoder = motor.getEncoder();
        
        // PID values need tuning for your specific elevator
        pid = new PIDController(0.1, 0, 0);
    }

    // Returns elevator height in inches
    public double getHeight() {
        return encoder.getPosition() / COUNTS_PER_INCH;
    }

    public void setPosition(double targetHeight) {
        double pidOutput = pid.calculate(getHeight(), targetHeight);
        
        // Add gravity compensation
        // The sign is positive because we need to work against gravity
        // You might need to flip the sign depending on your motor polarity
        double motorOutput = pidOutput + GRAVITY_COMPENSATION;
        
        // Clamp the output to valid range
        motorOutput = Math.min(Math.max(motorOutput, -1.0), 1.0);
        
        motor.set(motorOutput);  
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Good place to update SmartDashboard values if needed
    }
}