// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.constants.ConstantsElevator;

public class TrapezoidPIDElevator extends ProfiledPIDSubsystem {
  
  private static ProfiledPIDController profiledPIDController = new ProfiledPIDController(ConstantsElevator.Kp, 
  ConstantsElevator.Ki, ConstantsElevator.Kd, new TrapezoidProfile.Constraints(ConstantsElevator.MAX_VELOCITY, 
  ConstantsElevator.MAX_ACCELERATION));

  private VictorSP elevatorMotor_1, elevatorMotor_2;
  private SpeedControllerGroup elevatorMotorsGroup;

  private Encoder encoderElevator;
  private DigitalInput limitSwitchElevator_UP;
  private DigitalInput limitSwitchElevator_DOWN;

  private ElevatorFeedforward elevatorFeedforward;

  public TrapezoidPIDElevator(int ID_1, int ID_2, Encoder encoderElevator, 
  DigitalInput limitSwitchElevator_UP, DigitalInput limitSwitchElevator_DOWN) {
    super(profiledPIDController);

    this.elevatorMotor_1 = new VictorSP(ID_1);
    this.elevatorMotor_2 = new VictorSP(ID_2);
    this.encoderElevator = encoderElevator;
    this.limitSwitchElevator_UP = limitSwitchElevator_UP;
    this.limitSwitchElevator_DOWN = limitSwitchElevator_DOWN;
    this.elevatorMotorsGroup = new SpeedControllerGroup(elevatorMotor_1, elevatorMotor_2);
    this.elevatorFeedforward = new ElevatorFeedforward(ConstantsElevator.Ks, 
                                                       ConstantsElevator.Kg,
                                                       ConstantsElevator.Kv, 
                                                       ConstantsElevator.Ka);
    
    this.encoderElevator.setDistancePerPulse(ConstantsElevator.DISTANCE_PER_PULSE);
  }

  public Encoder getEncoder() {
    return encoderElevator;
  }

  public SpeedControllerGroup getElevatorMotorsGroup() {
    return elevatorMotorsGroup;
  }

  public DigitalInput getLimitSwitch_UP() {
    return limitSwitchElevator_UP;
  }

  public DigitalInput getLimitSwitch_DOWN() {
    return limitSwitchElevator_DOWN;
  }

  @Override
  public double getMeasurement() {
    return encoderElevator.getDistance();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = elevatorFeedforward.calculate(setpoint.position, setpoint.velocity);
    this.elevatorMotorsGroup.setVoltage(feedforward + output);
  }
}
