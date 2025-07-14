package org.firstinspires.ftc.teamcode.common.robot;



import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    HardwareMap mainMap;
    private Servo armServo = null;
    private Servo wristServo = null;
    private Servo clawServo = null;


    public Arm(HardwareMap hMap) {
        init(hMap);
    }

    public void init(HardwareMap hMap) {
        mainMap = hMap;

        armServo = hMap.get(Servo.class, "armServo");
        wristServo = hMap.get(Servo.class, "wristServo");
        clawServo = hMap.get(Servo.class, "clawServo");

        clawServo.setDirection(Servo.Direction.REVERSE);

        resetServos();

    }

//    public void cycleState1() {
//        armServo.setPosition(0.5);
//        wristServo.setPosition(0.5);
//        clawServo.setPosition(0.5);
//    }

    public void resetServos() {
        armServo.setPosition(0);
        wristServo.setPosition(0);
        clawServo.setPosition(0.01);
    }

    public void wristReset() {
        wristServo.setPosition(0);
    }
    public void wristTest() {
        wristServo.setPosition(1);

    }

    public void armReset() {
        armServo.setPosition(0);
    }

    public void armTest() {
        armServo.setPosition(0.5);

    }

    public void clawReset() {
        clawServo.setPosition(0.01);
    }
    public void clawTest() {
        clawServo.setPosition(0.28);
    }
}
