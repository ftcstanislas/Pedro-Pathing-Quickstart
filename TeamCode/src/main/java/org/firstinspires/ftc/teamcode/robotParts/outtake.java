package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class outtake {

    public DcMotorEx outtakeLeft, outtakeRight;

    private Servo claw,intake;

    public void init(HardwareMap map) {
        outtakeLeft = map.get(DcMotorEx.class, "outtakeLeft");
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeRight = map.get(DcMotorEx.class, "outtakeRight");
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void claw(double position){claw.setPosition(position);}

    public void moveOuttake(double outtakePowerLocal){
        outtakeLeft.setPower(outtakePowerLocal);
        outtakeRight.setPower(outtakePowerLocal);
    }
}
