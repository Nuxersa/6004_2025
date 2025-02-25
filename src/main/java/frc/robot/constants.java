package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class constants {

    public static final class ElevatorConstants {
        public static final int LIFT_MAIN = 62;             //PWM
        public static final int LIFT_FOLLOW = 61;             //PWM
        public static final int LIFT_CUR_LMT = 50;   //const

        public static final double LIFT_MOTOR_VOLTAGE_COMP = 10;
        public static final double LIFT_SPEED_DOWN = 0.5;
        public static final double LIFT_SPEED_UP = -0.5;
        public static final double LIFT_HOLD_DOWN = (0.5 * 0.05);
        public static final double LIFT_HOLD_UP = (0.5 * 0.05);

        public static final double LIFT_HEIGHT_1 = .1;   //set point 1
        public static final double LIFT_HEIGHT_2 = .2;   //set point 2
        public static final double LIFT_HEIGHT_3 = .3;   //set point 3

        public static final double   kElevatorGearing         = 12.0;
        public static final double   kElevatorSproketTeeth    = 22;
        public static final double   kElevatorPitch           = Units.inchesToMeters(0.25);
        public static final double   kElevatorDrumRadius      = (kElevatorSproketTeeth * kElevatorPitch) / (2 * Math.PI);// radius = Circumference / (2 pi)
        // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
        public static final double   kMinElevatorHeightMeters = Units.inchesToMeters(0);//min height / 10
        public static final double   kMaxElevatorHeightMeters = Units.inchesToMeters(30);

      }
   
    public final class TalonFXConstants {
        public static final int COUNTS_PER_REV = 2048;
        public static final double COUNTS_PER_DEG = COUNTS_PER_REV / 360.0;
    
        public static final boolean TALON_FUTURE_PROOF = true; 
        
    
        
    }
    
    public final class IntakeConstants {
      public static final int INTAKE_Pivot_ID=30;
      public static final int INTAKE_Grab_ID= 16;
      
      
      public static final double INTAKE_SPEED= .15;
      public static final double INTAKE_SPEED_HOLD= .0;
  
      public static final InvertedValue INTAKE_INVERSION = InvertedValue.Clockwise_Positive;
      public static final NeutralModeValue INTAKE_NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final double INTAKE_POSITION_STATUS_FRAME = 0.05;
      public static final double INTAKE_VELOCITY_STATUS_FRAME = 0.01;      
  
  }
}
