//<<<<<<< HEAD
//=======
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Intento de control PI :D
// Intento comentario 2 :O

//>>>>>>> 372b3c779d156f420adc045392477f160db6b4f3
// ivanna tortuga estuvo aqui
package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Encoder;

import java.sql.Time;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;;

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
 double setpoint = 0;
 //Kp = número proporcional 
 double Kp = 0.025; double P_Error_L; double P_Error_R;
 //Ki = número integral
 double Ki = 0.18; double I_Error_L=0; double I_Error_R=0; double LastTime = 0;
 //kD = número derivativo
 double Kd = 0.002; double D_Error_L=0; double D_Error_R=0; double LastErrorL = 0; double LastErrorR = 0;

  @Override
  public void robotInit() {

    //Se resetea variables y encoders
    ChasisEncodeR.reset(); ChasisEncodeL.reset();
    ChasisEncodeR.setReverseDirection(true);

    //Configura todos los motores en Brake mode al recibir 0
    MotorLeftS.setIdleMode(CANSparkMax.IdleMode.kBrake);     MotorLeftM.setIdleMode(CANSparkMax.IdleMode.kBrake);
    MotorRightS.setIdleMode(CANSparkMax.IdleMode.kBrake);    MotorRightM.setIdleMode(CANSparkMax.IdleMode.kBrake);

    //Calibra la distancia por pulso del encoder (1 vuelta es 128 pulsos en los Grayhill)
    ChasisEncodeR.setDistancePerPulse(Math.PI*4/128); //Math.PI/384
    ChasisEncodeL.setDistancePerPulse(Math.PI*4/128); //Math.PI/384

    //Set motores en 0 y designamos motores esclavos e inversos
    MotorLeftS.follow(MotorLeftM); MotorRightS.follow(MotorRightM);
    MotorRightM.set(0);  MotorLeftM.set(0);
    MotorRightM.setInverted(true);
  }

  @Override
  public void robotPeriodic() {

    //Reporte de valores chasis y joystick
    SmartDashboard.putNumber("Encoder Derecho", ChasisEncodeR.getDistance());
    SmartDashboard.putNumber("Encoder Izquierdo", ChasisEncodeL.getDistance());
    SmartDashboard.putNumber("Velocidad Motor Derecho", MotorRightM.get());
    SmartDashboard.putNumber("Velocidad Motor Izquierdo", MotorLeftM.get());
    SmartDashboard.putBoolean("A BUTTON", JoyDrive.getAButton());
    SmartDashboard.putBoolean("B BUTTON", JoyDrive.getBButton());
  }
  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}
  
  @Override
  public void teleopInit() {

    //Reset de variables para PID
    ChasisEncodeL.reset(); ChasisEncodeR.reset();
    LastTime = Timer.getFPGATimestamp();
    setpoint = 0;
    P_Error_R = 0; P_Error_L = 0;
    I_Error_L = 0; I_Error_R = 0;
    D_Error_L = 0; D_Error_R = 0;
  }

  @Override
  public void teleopPeriodic() {
    
    //Datos Robot y Joystick
    if(JoyDrive.getAButton()){setpoint = 100;}
    if(JoyDrive.getBButton()){setpoint = 1;}
    double IntegralZone = setpoint * 0.1;
    double posicionL= ChasisEncodeL.getDistance();
    double posicionR= ChasisEncodeR.getDistance();

    //kP Error Proporcional
    P_Error_L = setpoint-posicionL; 
    P_Error_R = setpoint-posicionR;

    //kI Error Integral 
    double dt = Timer.getFPGATimestamp() - LastTime;
    if(Math.abs(P_Error_L) < IntegralZone){I_Error_L += P_Error_L*dt;}
    if(Math.abs(P_Error_R) < IntegralZone){I_Error_R += P_Error_R*dt;}

    //kD Error derivativo
    if(Math.abs(P_Error_L) < IntegralZone){D_Error_L = (P_Error_L - LastErrorL) / dt; }
    if(Math.abs(P_Error_R) < IntegralZone){D_Error_R = (P_Error_R - LastErrorR) / dt; }
    

    //Control PID
    double velocidadL = (Kp*P_Error_L) + (Ki*I_Error_L) + (Kd*D_Error_L);
    double velocidadR = (Kp*P_Error_R) + (Ki*I_Error_R) + (Kd*D_Error_R);
    
    //Control de error
    if(Math.abs(setpoint-posicionL) > setpoint*0.0){MotorLeftM.set(velocidadL);}
    else{MotorLeftM.set(0);}
    if(Math.abs(setpoint-posicionR) > setpoint*0.0){MotorRightM.set(velocidadR);}
    else{MotorRightM.set(0);}

    //Control del tiempo
    LastTime = Timer.getFPGATimestamp();

    //Control error
    LastErrorL = P_Error_L; 
    LastErrorR = P_Error_R;
  }
 
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
