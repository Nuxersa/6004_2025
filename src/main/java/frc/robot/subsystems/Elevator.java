/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Elevator extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  SparkMax leftLeader;
  //SparkMax leftFollower;


  public Elevator() {
    leftLeader = new SparkMax(constants.LIFT_MAIN, MotorType.kBrushless);
    //leftFollower = new SparkMax(2, MotorType.kBrushless);  
    /*
     * Create new SPARK MAX configuration objects. These will store the
     * configuration parameters for the SPARK MAXes that we will set below.
     */
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    //SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    
    /*
     * Set parameters that will apply to all SPARKs. We will also use this as
     * the left leader config.
     */
    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);
    
    // Apply the global config and set the leader SPARK for follower mode
    //leftFollowerConfig
    //    .apply(globalConfig)
    //    .follow(leftLeader);

    // Apply the global config and set the leader SPARK for follower mode
    //leftFollowerConfig
    //    .apply(globalConfig)
    //    .follow(leftLeader);
    
    /*
     * Apply the configuration to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

  }


  public Command moveElevator(double speed) {
    
    return runOnce(
            () -> { 
              leftLeader.set(speed);
        });

  } 


}
