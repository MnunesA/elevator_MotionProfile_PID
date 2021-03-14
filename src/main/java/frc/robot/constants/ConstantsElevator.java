// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

/**
 * The ConstantsElevator class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ConstantsElevator {

   public static final double 
       Kp = 0,
       Ki = 0,
       Kd = 0,
       Ks = 0, 
       Kg = 0, 
       Kv = 0, 
       Ka = 0;

   public static final double 
       MAX_VELOCITY = 0, 
       MAX_ACCELERATION = 0, 
       INITIAL_POSITION = 0;

   public static final int 
       ENCODER_FINAL = 200,
       ENCODER_INITIAL = 10;                        

   public static final double DISTANCE_PER_PULSE = 0;
   public static final boolean ENCODER_INVERTED = false;

   public static final Encoder encoderElevator = new Encoder(Constants.CHANNEL_DIO_7, Constants.CHANNEL_DIO_8, 
                                                             ENCODER_INVERTED, Encoder.EncodingType.k2X);
   
   public static final DigitalInput 
       limitSwitch_UP = new DigitalInput(Constants.CHANNEL_DIO_0),
       limitSwitch_DOWN = new DigitalInput(Constants.CHANNEL_DIO_1);
}

