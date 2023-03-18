package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsREV extends SubsystemBase{

    int armStage = 0;

    //Pneumatics
    Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
    boolean enabled = phCompressor.isEnabled();
    boolean pressureSwitch = phCompressor.getPressureSwitchValue();
    double current = phCompressor.getPressure();
    //Switche
    DoubleSolenoid claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, 14, 15);
   DoubleSolenoid armTop = new DoubleSolenoid(PneumaticsModuleType.REVPH, 12, 13);
   DoubleSolenoid armBottom = new DoubleSolenoid(PneumaticsModuleType.REVPH, 10, 11);

   Value kReverse = DoubleSolenoid.Value.kReverse;
   Value kForward = DoubleSolenoid.Value.kForward;

   
    public PneumaticsREV(){
        phCompressor.enableAnalog(110, 120);
        claw.set(kReverse);
        armBottom.set(kReverse);
        armTop.set(kReverse);
       //phCompressor.disable();
    }
    

    public void toggleClaw(){
        claw.toggle();
    }

    int bottom = 0;
    int top = 0;

    public void setArmStage(double value){
        putPressure();
        if(value < -0.2){
            armBottom.set(kForward);
            armTop.set(kForward);
        } else if (value < 0.8){
            armBottom.set(kForward);
            armTop.set(kReverse);
        } else{
            armBottom.set(kReverse);
            armTop.set(kReverse);
        }
    }




    public void putPressure(){
         SmartDashboard.putNumber("Pressure", phCompressor.getPressure());
    }
}
