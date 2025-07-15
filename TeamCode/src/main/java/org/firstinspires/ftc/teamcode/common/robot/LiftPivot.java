package org.firstinspires.ftc.teamcode.common.robot;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LiftPivot {
    HardwareMap mainMap;
    private DcMotorEx liftMotorL = null;
    private DcMotorEx liftMotorR = null;
    private DcMotorEx pivotMotorL = null;
    private DcMotorEx pivotMotorR = null;

    public enum LiftState {
        RETRACTED(0),
        BUCKET(500);
        public int encoderValue;

        LiftState(int encoderValue){
            this.encoderValue = encoderValue;
        }
    }

    public enum PivotState {
        IDLE(0),
        INTAKE(250),
        BUCKET(500);
        public int encoderValue;

        PivotState(int encoderValue){
            this.encoderValue = encoderValue;
        }
    }



    public LiftState liftState = LiftState.RETRACTED;
    public PivotState pivotState = PivotState.BUCKET;
    public LiftPivot(HardwareMap hMap) {
        init(hMap);
    }

    public void init(HardwareMap hMap) {
        mainMap = hMap;

        liftMotorL = hMap.get(DcMotorEx.class, "liftMotorL");
        liftMotorR = hMap.get(DcMotorEx.class, "liftMotorR");
        pivotMotorL = hMap.get(DcMotorEx.class, "pivotMotorR");
        pivotMotorR = hMap.get(DcMotorEx.class, "pivotMotorR");
    }
}