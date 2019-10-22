/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.ToggledSolenoid;
import frc.robot.subsystems.VisionTracking;
import frc.robot.subsystems.Winch;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.*;

public class Robot extends TimedRobot {
  public static Drivebase m_drivebase;
  public static OI m_oi;
  public static VisionTracking m_visiontracking;
  AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  Compressor c = new Compressor(0);
  
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  double previous_error;
  boolean shiftState = false;

  public static ToggledSolenoid winchPiston = new ToggledSolenoid(2, 5);
  public static ToggledSolenoid shifters = new ToggledSolenoid(3, 4);
  public static ToggledSolenoid hatchIntake = new ToggledSolenoid(6, 7);
  public static ToggledSolenoid intakeActuator = new ToggledSolenoid(0, 1);

  public static Spark leftWinch = new Spark(4);
  public static Spark rightWinch = new Spark(5);
  public static Winch winch = new Winch(leftWinch, rightWinch, winchPiston);

  @Override
  public void robotInit() {
    m_oi = new OI();
    m_drivebase = new Drivebase();
    m_visiontracking = new VisionTracking();
    m_chooser.setDefaultOption("Default Auto", new ArcadeDrive(m_oi.getDriveJoyY(), m_oi.getDriveJoyX()));
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    CameraServer.getInstance().startAutomaticCapture();

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    matchPeriodic();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    matchPeriodic();
    Scheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {
  }

  void matchPeriodic() {
    
    c.setClosedLoopControl(true);

    m_visiontracking.setTracking(true);

    //shifters
    if (m_oi.DRIVE_JOY.getRawButtonPressed(6)) {
      shifters.togglePiston();
      shiftState = !shiftState;
    }
    SmartDashboard.putString("Gear", shiftState ? "High" : "Low");

    //hatch intake
    // if (m_oi.DRIVE_JOY.getRawButtonPressed(4)) {
    //   hatchIntake.togglePiston();
    // }


  if (m_oi.DRIVE_JOY.getRawButtonPressed(3)){
    hatchIntake.set(true);
  }else if(m_oi.DRIVE_JOY.getRawButtonPressed(4)){
    hatchIntake.set(false);
  }
    
    //intake actuator
    // if (m_oi.DRIVE_JOY.getRawButtonPressed(3)) {
    //   intakeActuator.togglePiston();
    // }

     if (m_oi.getOpJoyBLPressed()) {
       intakeActuator.togglePiston();
     }

    //Winch piston
    if (m_oi.OP_JOY.getBButtonPressed()) {
      winchPiston.togglePiston();
    }

    //Winch control
    winch.setWinch(-m_oi.getOpJoyYL());

    //Field-oriented drive
    double desired;
    if (m_oi.getDriveJoyY() != 0) {
      desired = Math.toDegrees(Math.atan(m_oi.getDriveJoyX() / m_oi.getDriveJoyY()));
    } else {
      desired = 90;
    }

    double desiredCA;
    if (m_oi.getDriveJoyX() >= 0) {
      if (m_oi.getDriveJoyY() >= 0) {
        desiredCA = desired;
      } else {
        desiredCA = desired + 180;
      }

    } else {
      if (m_oi.getDriveJoyY() <= 0) {
        desiredCA = desired + 180;
      } else {
        desiredCA = desired + 360;
      }
    }

    int negCorrector = m_gyro.getYaw() < 0 ? 1 : 0;
    double gyroCA = m_gyro.getYaw() % 360 + negCorrector * 360;

    double direction = desiredCA < gyroCA ? 1 : -1;
    if (Math.abs(desiredCA - gyroCA) > Math.abs(desiredCA + 360 * direction - gyroCA)) {
      desiredCA += 360 * direction;
    }

    double error = desiredCA - gyroCA;
    double kP = 0.004; // .038;
    double deriv = (error - this.previous_error) / .02;
    double kD = .002;
    previous_error = error;

    //vision tracking

    //displaying vision target
     if (m_visiontracking.getTarget() == 1) {
      SmartDashboard.putBoolean("Target Acquired", true);
    } else {
      SmartDashboard.putBoolean("Target Acquired", false);
    }

    if (m_oi.getDriveTrigger()) {
      double steeringAdjust = m_visiontracking.pidX();
      m_drivebase.arcadeDrive(m_visiontracking.zoomForward(), steeringAdjust * m_visiontracking.getTarget());
      shifters.set(false);
      shiftState = false;
      if (m_visiontracking.get("ta") > 5) {
        intakeActuator.set(true);
      }

      //if (m_visiontracking.get("ta") > 6) {
         //shifters.set(true);
         //shiftState = true;
       //}
    } else {
      m_drivebase.arcadeDrive(m_oi.getControlJoyY(), m_oi.getDriveJoyMag()*(kP * error + kD * deriv + .25 * Math.signum(error)));
    }

    SmartDashboard.putNumber("Desired CA", desiredCA);
    SmartDashboard.putNumber("Current CA", gyroCA);

    //gyro reset
    if (m_oi.DRIVE_JOY.getRawButtonPressed(11)) {
      m_gyro.reset();
    }
  }

  public static void initTalon(TalonSRX motor) {

    motor.setNeutralMode(NeutralMode.Brake);
    motor.neutralOutput();
    motor.setSensorPhase(false);
    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    motor.configNominalOutputForward(0.0, 0);
    motor.configNominalOutputReverse(0.0, 0);

  }
}
