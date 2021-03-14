// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ConstantsElevator;
import frc.robot.subsystems.TrapezoidPIDElevator;

public class TrapezoidCommandSetGoal extends CommandBase {
  
  private TrapezoidPIDElevator tps;
  private TrapezoidProfile.State goal;
  private boolean limitSwitchStatus_UP;
  private boolean limitSwitchStatus_DOWN;
  private double position;
  private double velocity;
  private double currentPosition;
  private double currentVelocity;
  private double prevVelocity = 0.0;
  private double currentAcceleration;
  private double output;

  public TrapezoidCommandSetGoal(TrapezoidPIDElevator trapezoidProfileSubsystem, double position, double velocity) {
    tps = trapezoidProfileSubsystem;
    this.position = position;
    this.velocity = velocity;
    addRequirements(tps);
  }

  public TrapezoidCommandSetGoal(TrapezoidPIDElevator trapezoidProfileSubsystem, double position) {
    tps = trapezoidProfileSubsystem;
    this.position = position;
    this.velocity = 0.0;
    addRequirements(tps);
  }

  @Override
  public void initialize() {
    limitSwitchStatus_UP = !tps.getLimitSwitch_UP().get();
    limitSwitchStatus_DOWN = !tps.getLimitSwitch_DOWN().get();
    goal = new TrapezoidProfile.State(position, velocity);
    tps.setGoal(goal);
  }

  @Override
  public void execute() {
    this.currentVelocity = tps.getEncoder().getRate();
    this.currentAcceleration = (currentVelocity - prevVelocity)/0.02;
    this.prevVelocity = currentVelocity;
    this.tps.useOutput(this.output, new TrapezoidProfile.State(currentVelocity, currentAcceleration));
  }

  @Override
  public void end(boolean interrupted) {
    if (tps.getLimitSwitch_UP().get() == limitSwitchStatus_UP) {
      new TrapezoidCommandSetGoal(tps, ConstantsElevator.ENCODER_FINAL).schedule();

    } else if (tps.getLimitSwitch_DOWN().get() == limitSwitchStatus_DOWN){
      new TrapezoidCommandSetGoal(tps, ConstantsElevator.ENCODER_INITIAL).schedule();
      
    } else {
      tps.getElevatorMotorsGroup().set(0.0);
    }
  }

  @Override
  public boolean isFinished() {
    boolean stop = (currentPosition == goal.position) ||
    (tps.getLimitSwitch_UP().get() == limitSwitchStatus_UP) || 
    (tps.getLimitSwitch_DOWN().get() == limitSwitchStatus_DOWN);
    return stop;
  }
}
