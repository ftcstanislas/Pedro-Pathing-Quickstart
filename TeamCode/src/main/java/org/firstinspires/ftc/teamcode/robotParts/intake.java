package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class intake {

    public Servo wrist, scissor;

    public CRServo CRLeft,CRRight;

    public void init(HardwareMap map) {
        scissor = map.get(Servo.class,"scissor");
        scissor.setDirection(Servo.Direction.REVERSE);

        wrist = map.get(Servo.class, "wrist");

        CRLeft = map.get(CRServo.class, "intakeLeft");
        CRRight = map.get(CRServo.class,"intakeRight");
    }

    public void wrist(double position){wrist.setPosition(position);}
    public void setScissor(double position){scissor.setPosition(position);}
    public void run(double power){
        CRLeft.setPower(power);
        CRRight.setPower(-power);
    }
}
