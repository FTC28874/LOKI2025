package org.firstinspires.ftc.teamcode.common.robot;



import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    HardwareMap mainMap;
    private Servo armServo = null;
    private Servo wristServo = null;
    private Servo clawServo = null;

    public enum ArmState {
        IDLE(0.7),
        SPEC_INTAKE(0.37),
        SPEC_SCORE(0.02),
        BUCKET(0.32);
        public double armServoValue;

        ArmState(double armServoValue){
            this.armServoValue = armServoValue;
        }
    }

    public enum WristState {
        HORIZONTAL(0.38),

        VERTICAL(0.33);
        public double wristServoValue;

        WristState(Double wristServoValue){
            this.wristServoValue = wristServoValue;
        }
    }



    public ArmState armState = ArmState.IDLE;
    public WristState wristState = WristState.HORIZONTAL;
    public Arm(HardwareMap hMap) {
        init(hMap);
    }

    public void init(HardwareMap hMap) {
        mainMap = hMap;
//fd
        armServo = hMap.get(Servo.class, "armServo");
        wristServo = hMap.get(Servo.class, "wristServo");
        clawServo = hMap.get(Servo.class, "clawServo");

        clawServo.setDirection(Servo.Direction.REVERSE);

        setArmToState(armState);
        setWristToState(wristState);
        clawOpen();
    }
    public void setArmToState(ArmState targetState) {
        armState = targetState;
        setArmServoPosition(armState.armServoValue);
    }
    public void setWristToState(WristState targetState) {
        wristState = targetState;
        setWristServoPosition(wristState.wristServoValue);
    }
    public void setWristServoPosition(double servoPosition){
        wristServo.setPosition(servoPosition);
    }
    public void setArmServoPosition(double servoPosition){
        armServo.setPosition(servoPosition);
    }
    public void clawClose(){
        clawServo.setPosition(0.01);
    }
    public void clawOpen(){
        clawServo.setPosition(0.28);
    }
}