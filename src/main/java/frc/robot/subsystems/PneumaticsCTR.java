package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsCTR extends SubsystemBase{

    int armStage = 0;

    //Pneumatics
    Compressor phCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    boolean enabled = phCompressor.isEnabled();
    boolean pressureSwitch = phCompressor.getPressureSwitchValue();
    double current = phCompressor.getPressure();
    //Switche
    DoubleSolenoid claw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
   DoubleSolenoid armTop = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
   DoubleSolenoid armBottom = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

   Value kReverse = DoubleSolenoid.Value.kReverse;
   Value kForward = DoubleSolenoid.Value.kForward;

   
    public PneumaticsCTR(){
        phCompressor.enableDigital();
        claw.set(kReverse);
        armBottom.set(kReverse);
        armTop.set(kReverse);
       // phCompressor.disable();
    }
    

    public void toggleClaw(){
        claw.toggle();
    }

    int bottom = 0;
    int top = 0;

    public void setArmStage(double value){
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




    public double getPressure(){
        return phCompressor.getPressure();
    }
}
