package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class intake {

    public Servo wrist;
    private CRServo intake;

    public void init(HardwareMap map) {
        wrist = map.get(Servo.class, "wrist");
        intake = map.get(CRServo.class, "crservo");
    }

    public void wrist(double position){wrist.setPosition(position);}
    public void run(double power){intake.setPower(power);}
}
