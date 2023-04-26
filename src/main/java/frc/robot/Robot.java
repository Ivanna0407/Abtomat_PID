// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Intento de control PI :D

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

 private final XboxController JoyDrive = new XboxController(0);
 //Build de Motocontroladores (Motores)
 private final CANSparkMax MotorRightM = new CANSparkMax(1, MotorType.kBrushless);
 private final CANSparkMax MotorRightS = new CANSparkMax(2, MotorType.kBrushless);
 private final CANSparkMax MotorLeftM = new CANSparkMax(3, MotorType.kBrushless); 
 private final CANSparkMax MotorLeftS = new CANSparkMax(4, MotorType.kBrushless);
 
 //Build de encoders y set de distancia chasis
 Encoder ChasisEncodeR = new Encoder(6, 7);
 Encoder ChasisEncodeL = new Encoder(8, 9); 

 //PID
 //Kp= número proporcional 
 double Kp = 0.05;
 double setpoint = 0;
 //Ki=número integral
 double Ki = 0.02;
 double sumaerrorL=0; 
 double sumaerrorR=0;
  @Override
  public void robotInit() {
    //Se resetea variables y encoders
    ChasisEncodeR.reset(); ChasisEncodeL.reset();
    ChasisEncodeR.setReverseDirection(true);
    //Configura todos los motores en Brake mode al recibir 0
    MotorLeftS.setIdleMode(CANSparkMax.IdleMode.kBrake);     MotorLeftM.setIdleMode(CANSparkMax.IdleMode.kBrake);
    MotorRightS.setIdleMode(CANSparkMax.IdleMode.kBrake);    MotorRightM.setIdleMode(CANSparkMax.IdleMode.kBrake);

    //Calibra la distancia por pulso del encoder (1 vuelta es 128 pulsos en los Grayhill)
    ChasisEncodeR.setDistancePerPulse(Math.PI/384);
    ChasisEncodeL.setDistancePerPulse(Math.PI/384);

    //Set motores en 0 y designamos motores esclavos
    MotorLeftS.follow(MotorLeftM); MotorRightS.follow(MotorRightM);
    MotorRightM.set(0);  MotorLeftM.set(0);
    MotorRightM.setInverted(true);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder Derecho", ChasisEncodeR.getDistance());
    SmartDashboard.putNumber("Encoder Izquierdo", ChasisEncodeL.getDistance());
    SmartDashboard.putNumber("Velocidad Motor Derecho", MotorRightM.get());
    SmartDashboard.putNumber("Velocidad Motor Izquierdo", MotorLeftM.get());
    SmartDashboard.putBoolean("A BUTTON", JoyDrive.getAButton());
    SmartDashboard.putBoolean("B BUTTON", JoyDrive.getBButton());
  }
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}
  
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    ChasisEncodeL.reset(); ChasisEncodeR.reset();
    setpoint = 0;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(JoyDrive.getAButton()){
      setpoint = 10;
    }
    if(JoyDrive.getBButton()){
      setpoint = 0;
    }
    //datos del encoder 
    double posicionL= ChasisEncodeL.getDistance();
    double posicionR= ChasisEncodeR.getDistance();
    //Suma los errores para avanzar un poco 
    sumaerrorL=sumaerrorL + (setpoint-posicionL);
    sumaerrorR=sumaerrorR + (setpoint-posicionR);
    //calculo////////////////////////////////////////////////////////
    //Calcula cuanto falta para llegar a en este caso 100 pulgadas KP
    double velocidadL = Kp*(setpoint-posicionL)+ Ki*sumaerrorL;
    double velocidadR = Kp*(setpoint-posicionR)+ Ki*sumaerrorR;
    if(setpoint-posicionL > setpoint*0.05){
      MotorLeftM.set(velocidadL);
    }else{
      MotorLeftM.set(0);
    }
      
    if(setpoint-posicionR > setpoint*0.05){
      MotorRightM.set(velocidadR);
    }else{
      MotorRightM.set(0);
    }
    SmartDashboard.putNumber("ErrorL", velocidadL/Kp);
    SmartDashboard.putNumber("ErrorR", velocidadR/Kp);
  }
 
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}