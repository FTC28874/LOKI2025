package org.firstinspires.ftc.teamcode.common.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class LiftPivot {

    private DcMotorEx liftMotorL = null;
    private DcMotorEx liftMotorR = null;
    private DcMotorEx pivotMotorL = null;
    private DcMotorEx pivotMotorR = null;

    private static final double TICKS_PER_DEG = 26.13;

    // PID constants
    private static final double kP_PIVOT = 0.008, kI_PIVOT = 0.00, kD_PIVOT = 0.0003;
    private static final double kP_LIFT  = 0.004, kI_LIFT  = 0.00, kD_LIFT  = 0.0002;

    private static final double GRAVITY_FF_NOMINAL = 0.05;
    private static final double LIFT_SAG_GAIN = -3e-4;

    // Motion limits
    private static final int MIN_PIVOT = -50, MAX_PIVOT = 600;
    private static final int MIN_LIFT  = 0, MAX_LIFT = 2280;

    // Internal PID state
    private double targetPivot = 0, targetLift = 0;
    private double iPivot = 0, prevErrPivot = 0;
    private double iLift = 0, prevErrLift = 0;
    private int intakeStage = 0;
    private boolean lastIntakeButton = false;
    private ElapsedTime dtTimer = new ElapsedTime();

    public enum LiftState {
        RETRACTED(0),
        BUCKET(2250),
        MANUAL_ADJUST(1200),
        SPEC_INTAKE(156),
        SPEC_SCORE(1300),
        SPEC_HANG(1750),
        INTAKE_MID(1200);

        public final int encoderValue;

        LiftState(int encoderValue) {
            this.encoderValue = encoderValue;
        }
    }

    public enum PivotState {
        IDLE(0),
        INTAKE(480),
        LOWERED_INTAKE(530),
        CLIMB(250),
        SPEC_INTAKE(475),
        BUCKET(-50);

        public final int encoderValue;

        PivotState(int encoderValue) {
            this.encoderValue = encoderValue;
        }
    }

    public LiftState liftState = LiftState.RETRACTED;
    public PivotState pivotState = PivotState.IDLE;

    public LiftPivot(HardwareMap hMap, boolean resetEncoder) {
        init(hMap, resetEncoder);
    }

    public void init(HardwareMap hMap, boolean resetEncoder) {
        liftMotorL = hMap.get(DcMotorEx.class, "liftMotorL");
        liftMotorR = hMap.get(DcMotorEx.class, "liftMotorR");
        pivotMotorL = hMap.get(DcMotorEx.class, "pivotMotorL");
        pivotMotorR = hMap.get(DcMotorEx.class, "pivotMotorR");

        for (DcMotorEx motor : new DcMotorEx[]{liftMotorL, liftMotorR, pivotMotorL, pivotMotorR}) {
            if (resetEncoder) motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        pivotMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        dtTimer.reset();
    }

    public void update() {
        double dt = dtTimer.seconds();
        dtTimer.reset();
        if (liftState == LiftState.MANUAL_ADJUST) {
            targetLift = targetLift;
        } else {
            targetLift = Range.clip(liftState.encoderValue, MIN_LIFT, MAX_LIFT);
        }
        targetPivot = Range.clip(pivotState.encoderValue, MIN_PIVOT, MAX_PIVOT);

        // === Pivot PID + Gravity FF ===
        double posPivot = pivotMotorL.getCurrentPosition();
        double errPivot = targetPivot - posPivot;
        iPivot += errPivot * dt;
        double dPivot = (errPivot - prevErrPivot) / dt;
        prevErrPivot = errPivot;

        double thetaRad = posPivot / TICKS_PER_DEG * Math.PI / 180.0;
        double sag = (liftState == LiftState.BUCKET) ? 0.0 : LIFT_SAG_GAIN * liftMotorL.getCurrentPosition();
        double gravityFF = (GRAVITY_FF_NOMINAL + sag) * Math.cos(thetaRad);

        double outPivot = kP_PIVOT * errPivot + kI_PIVOT * iPivot + kD_PIVOT * dPivot + gravityFF;
        if (getLiftPosition() < LiftState.SPEC_HANG.encoderValue + 100) {
            outPivot = Range.clip(outPivot, -0.5, 0.5);
        } else {
            outPivot = Range.clip(outPivot, -1, 1);
        }
        pivotMotorL.setPower(outPivot);
        pivotMotorR.setPower(outPivot);

        // === Lift PID ===
        double posLift = liftMotorL.getCurrentPosition();
        double errLift = targetLift - posLift;
        iLift += errLift * dt;
        double dLift = (errLift - prevErrLift) / dt;
        prevErrLift = errLift;

        double outLift = kP_LIFT * errLift + kI_LIFT * iLift + kD_LIFT * dLift;
        outLift = Range.clip(outLift, -1, 1);
        liftMotorL.setPower(outLift);
        liftMotorR.setPower(outLift);
    }

    // Optional: set targets from opmode
    public void setLiftState(LiftState newState) {
        this.liftState = newState;
    }

    public void setPivotState(PivotState newState) {
        this.pivotState = newState;
    }

    public int getPivotPosition() {
        return pivotMotorL.getCurrentPosition();
    }

    public int getLiftPosition() {
        return liftMotorL.getCurrentPosition();
    }

    public boolean isLiftRetractedForPivot() {
        return getLiftPosition() <= 30; // threshold value
    }

    public void adjustLiftTarget(int deltaTicks) {
        liftState = LiftState.MANUAL_ADJUST;
        targetLift = Range.clip(targetLift + deltaTicks, MIN_LIFT, MAX_LIFT);
        //liftState = null; // prevent overriding by a preset
    }
}
