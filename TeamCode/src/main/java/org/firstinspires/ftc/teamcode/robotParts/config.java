package org.firstinspires.ftc.teamcode.robotParts;

public class config {
    //TODO: maybe just name things after their port and change that over here
    public enum motors {
        leftBack("left_back"),//CH0
        leftFront("left_front"),//CH1
        rightBack("right_back"),//CH3
        rightFront("right_front"),//CH2

        arm("arm"),//EH0
        hookLeft("hookLeft"),//EH1
        hookRight("hookRight"),//EH2
        slides("slides");//EH3

        private final String name;

        public String getName() {return this.name;}

        motors(String name) {this.name = name;}
    }
    public enum servos {
        wristLeft("wristLeft"),//CH1
        intake("intake"),//CH2
        claw("claw"),//CH4

        wristRight("wristRight"),//EH0
        keepSlide("keepSlides");//EH1

        private final String name;

        public String getName() {return this.name;}

        servos(String name) {this.name = name;}
    }
}
