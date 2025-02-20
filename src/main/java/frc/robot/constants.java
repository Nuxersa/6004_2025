package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class constants {

    public static final class ElevatorConstants {
        public static final int LIFT_MAIN = 62;             //PWM
        public static final int LIFT_FOLLOW = 7;             //PWM
        public static final int LIFT_CUR_LMT = 50;   //const

        public static final double LIFT_MOTOR_VOLTAGE_COMP = 10;
        public static final double LIFT_SPEED_DOWN = 0.4;
        public static final double LIFT_SPEED_UP = -0.4;
        public static final double LIFT_HOLD_DOWN = 0.1;
        public static final double LIFT_HOLD_UP = -0.15;
      }
   
    public final class TalonFXConstants {
        public static final int COUNTS_PER_REV = 2048;
        public static final double COUNTS_PER_DEG = COUNTS_PER_REV / 360.0;
    
        public static final boolean TALON_FUTURE_PROOF = true; 
        
    
        
    }
    
    public final class IntakeConstants {
      public static final int INTAKE_FRONT_ID=20;
      public static final int INTAKE_BACK_ID= 2;
      public static final int INTAKE_MIDDLE_ID= 1;
      
      public static final double INTAKE_SPEED= .3;
      public static final double INTAKE_SPEED_HOLD= .0;
  
      public static final InvertedValue INTAKE_INVERSION = InvertedValue.Clockwise_Positive;
      public static final NeutralModeValue INTAKE_NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final double INTAKE_POSITION_STATUS_FRAME = 0.05;
      public static final double INTAKE_VELOCITY_STATUS_FRAME = 0.01;
  
  }
}
