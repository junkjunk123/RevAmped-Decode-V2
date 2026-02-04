package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

@Config
public class ServoTurret extends HwServo {
    public enum turretPresets{
        LEFT_180(0/255f),
        LEFT_135(32/255f),
        LEFT_90(64/255f),
        LEFT_45(96/255f),
        CENTER(128/255f),
        RIGHT_45(159/255f),
        RIGHT_90(191/255f),
        RIGHT_135(223/255f),
        RIGHT_180(255/255f);
        private float pos;
        turretPresets(float pos) {
            this.pos = pos;
        }
        public float getPos(){
            return this.pos;
        }
    }
    turretPresets currentPreset = turretPresets.CENTER;
    HwServo turret_2;
    public ServoTurret(HardwareMap hardwareMap) {
        super(hardwareMap,"turret_1");
        turret_2 = new HwServo(hardwareMap,"turret_2");
    }

    public void setPosition(float pos){
        super.setPosition(pos);
        turret_2.setPosition(pos);
    }

    public void setPreset(turretPresets preset){
        currentPreset = preset;
        setPosition(currentPreset.getPos());
    }

    public void next(){
        if (currentPreset.ordinal() < turretPresets.values().length){
            setPreset(turretPresets.values()[currentPreset.ordinal()+1]);
        }
    }

    public void previous(){
        if (0 < currentPreset.ordinal()){
            setPreset(turretPresets.values()[currentPreset.ordinal()-1]);
        }
    }

    public void update(){
        super.update();
        turret_2.update();
    }

}
