package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{

    CANSparkMax m_motor;
    SparkMaxPIDController m_pidController;
    RelativeEncoder m_Encoder;

    DigitalInput m_limmitTop;
    DigitalInput m_limmitBottom;

    

    public ElevatorSubsystem(){

        m_motor = new CANSparkMax(62, MotorType.kBrushless);
        m_pidController = m_motor.getPIDController();
        
        m_Encoder = m_motor.getEncoder();
        m_pidController.setFeedbackDevice(m_Encoder);

        m_limmitTop = new DigitalInput(0);
        m_limmitBottom = new DigitalInput(1);

    }

    // Encoder object created to display position values
    
   
    public void drive(double value){
        Double voltage = -value*0.5;
        SmartDashboard.putBoolean("Top Limmit", m_limmitTop.get());
        SmartDashboard.putBoolean("Bottom Limmit", m_limmitBottom.get());
        if(voltage > 0){
            if(m_limmitTop.get()) m_pidController.setReference(0, CANSparkMax.ControlType.kVoltage);
            else m_pidController.setReference(voltage, CANSparkMax.ControlType.kVoltage);
        } else {
            if(m_limmitBottom.get()) m_pidController.setReference(0, CANSparkMax.ControlType.kVoltage);
            else m_pidController.setReference(voltage, CANSparkMax.ControlType.kVoltage);
        }
    }
}
