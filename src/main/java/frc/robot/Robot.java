/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.CANTalonFactory;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myrobot;
  private SpeedControllerGroup left;
  private SpeedControllerGroup right;
  private PWMVictorSPX sucker;
  private TalonSRX motorEncoder;
  private Spark led1;
  private Spark led2;

  private CameraServer camserv;
  // The larger the gear reduction the better
  // should stop at 120
  // make sure to have a pressure switch/pressure relief valve
  private Compressor compressor;
  // should be at most 60
  private DoubleSolenoid dsol1;
  private DoubleSolenoid dsol2;

  private UsbCamera _cam;

  private int count1;
  private int count2;

  private double kLiftSoftLimit = 39;
  private double kLiftTicksPerInch = 652; //4096 is ticks per revolution need to convert to inches
  private double liftP = .7;
  private double liftI = 0;
  private double liftD = 0;
  private double kLiftTolerance = .5;

  private XboxController controller;
  private XboxController controller2;
  // private PowerDistributionPanel pdp;
  // Inversion
  // MotorMagic
  // LiftTicksPerInch needs to be converted

  /*
  Helpful sources:
  Chief Delphi
  Programming done right frc
  StackOverFlow (ofc)
  Logan Buyers
  Head First Java
  ScreenStepsLive
  */

  // PID loop set f to 0 highly recommended

  @Override
    public void robotInit() {
    // _cam = CameraServer.getInstance().startAutomaticCapture();
    // camserv.gt
    camserv = CameraServer.getInstance();
    camserv.startAutomaticCapture(0);
    camserv.startAutomaticCapture(1);
    // pdp = new PowerDistributionPanel(0);
    // compressor = new Compressor();
    // dsol1 = new DoubleSolenoid(0, 1);
    // dsol2 = new DoubleSolenoid(2, 3);

    left = new SpeedControllerGroup(new PWMVictorSPX(7));//, new PWMVictorSPX(1));
    right = new SpeedControllerGroup(new PWMVictorSPX(8));//, new PWMVictorSPX(3));
    sucker = new PWMVictorSPX(9);
    m_myrobot = new DifferentialDrive(left, right);
    led1 = new Spark(6);
    // led1.set(.65);

    count1 = 1;
    count2 = 1;

    motorEncoder = CANTalonFactory.createTalon(1, false, NeutralMode.Brake,
    FeedbackDevice.QuadEncoder, 0, true);
    motorEncoder = CANTalonFactory.setupHardLimits(motorEncoder, LimitSwitchSource.Deactivated,
    LimitSwitchNormal.Disabled, false, LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, false);
    motorEncoder = CANTalonFactory.setupSoftLimits(motorEncoder, true, (int) Math.round(kLiftSoftLimit*kLiftTicksPerInch),
    true, 0);
    motorEncoder = CANTalonFactory.tuneLoops(motorEncoder, 0, liftP, liftI, liftD, 0);
    motorEncoder.setSelectedSensorPosition(0); //LIFT ALL THE WAY DOWN ON POWER UP

    controller = new XboxController(0);
    controller2 = new XboxController(1);

    System.out.println("Robot initialized");
  }

  @Override
  public void teleopInit() {
    Init();
    
  }

  @Override
  public void disabledInit() {
    // compressor.stop();
    motorEncoder.set(ControlMode.Disabled, 0);
  }

  public void Init()
  {
    // compressor.start();
    // sucker.setSafetyEnabled(true);
  }

  public void periodic()
  {
    led1.set(-0.97);
    liftUpdater();
    m_myrobot.tankDrive(-(controller.getY(Hand.kLeft)*Math.abs(controller.getY(Hand.kLeft))*.8), -(controller.getY(Hand.kRight)*Math.abs(controller.getY(Hand.kRight))*.8));
    // m_myrobot.tankDrive(-(controller.getY(Hand.kLeft)*Math.abs(controller.getY(Hand.kLeft)*.5), -controller.getY(Hand.kRight)*Math.abs(controller.getY(Hand.kRight))*.5);
    // System.out.println(pdp.getCurrent(7) + ", "  + pdp.getCurrent(9));

    m_myrobot.feedWatchdog(); 

    if (controller.getBumper(Hand.kLeft))
    {
      sucker.set(.7);
    }
    else if (controller.getBumper(Hand.kRight))
      sucker.set(-.5);
    else
      sucker.set(0);

    // if (controller.getBumperPressed(Hand.kRight))
    // {
    //   System.out.println(count1, " Count1");
    //   if (count1 % 2 == 0)
    //   {
    //     dsol1.set(DoubleSolenoid.Value.kReverse);
    //   }
    //   else
    //   {
    //     dsol1.set(DoubleSolenoid.Value.kForward);
    //   }
    //   // Enable / disable solenoid front
    //   count1++;
    // }
    // else if (controller.getBumperPressed(Hand.kLeft))
    // {
    //   System.out.println(count2 + " count2");
    //   // Enable / disable solenoid back
    //   if (count2 % 2 == 0)
    //   {
    //     System.out.println("count % 2");
    //     dsol2.set(DoubleSolenoid.Value.kReverse);
    //   }
    //   else
    //   {
    //     System.out.println("elsed");
    //     dsol2.set(DoubleSolenoid.Value.kForward);
    //   }
    //   count2++;
    // }
  }

  @Override
  public void autonomousInit()
  {
    Init();
  }

  @Override
  public void autonomousPeriodic()
  {
    periodic();
  }

  @Override
  public void teleopPeriodic()
  {
    periodic();
  }

  private void liftUpdater()
  {
    if (controller.getTriggerAxis(Hand.kLeft) > 0)
    {
      jog(controller.getTriggerAxis(Hand.kLeft)*.1);
      // motorEncoder.set(ControlMode.Position, -controller.getTriggerAxis(Hand.kLeft));
     
    }

    else if (controller.getTriggerAxis(Hand.kRight) > 0)
    {
      // motorEncoder.(ControlMode.Position, controller.getTriggerAxis(Hand.kRight));
      jog(-controller.getTriggerAxis(Hand.kRight)*.1);
    }

    if (controller.getAButtonPressed())
      setPosition(0);
    else if (controller.getBButtonPressed())
      // tune for the middle height
      setPosition(27.5);
    else if (controller.getYButtonPressed())
      // tune for top height
      setPosition(39);

    positionUpdater();
  }

   //CLOSED LOOP CONTROL
   private double mWantedPosition = .1;
   private double mTravelingPosition = 0;
   
   public synchronized void setPosition(double pos){
       if(pos>=kLiftSoftLimit)pos=kLiftSoftLimit;
       else if(pos<0)pos=0;
       mWantedPosition=pos;
       
      // System.out.println("Set wanted pos to "+pos);
   }

   private boolean jog=false;

   public void jog(double amount){
       setPosition(mWantedPosition+=amount);
       jog=true;
   }


  public boolean atPosition(){
     if(Math.abs(mWantedPosition-getPosition())<=kLiftTolerance){
         return true;
     }else{
         return false;
     }
      
  }

  public double getPosition(){
      return motorEncoder.getSelectedSensorPosition()/kLiftTicksPerInch;
  }

  private void positionUpdater(){
   if(mWantedPosition!=mTravelingPosition){
       mTravelingPosition=mWantedPosition;
       if(!jog) System.out.println("Lift to "+mTravelingPosition);
       jog=false;
       motorEncoder.set(ControlMode.Position, mTravelingPosition*kLiftTicksPerInch);
      //  System.out.println(mTravelingPosition);
    } 
  }

  private void setLimitClear(boolean e)
  {
    motorEncoder.configClearPositionOnLimitR(e, 0);
  }
}
