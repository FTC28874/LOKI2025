package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Pivot + Lift PID/FF Test", group = "Tests")
public class PivotLiftPIDTest extends LinearOpMode {

    /* ----------------------------------------------------------------
       HARDWARE
     ---------------------------------------------------------------- */
    private DcMotorEx pivotL, pivotR;
    private DcMotorEx liftL,  liftR;

    /* ----------------------------------------------------------------
       CONSTANTS you will tune ↓↓↓
     ---------------------------------------------------------------- */
    // –– Ticks / deg for your encoder‑gearbox chain (≈ 288 :1 goBilda HD).
    private static final double TICKS_PER_DEG = 26.13;

    // –– PID gains (rough starting points)
    private static final double kP_PIVOT = 0.008,  kI_PIVOT = 0.00,   kD_PIVOT = 0.0003;
    private static final double kP_LIFT  = 0.004,  kI_LIFT  = 0.00,   kD_LIFT  = 0.0002;

    // –– Feed‑forward for gravity on pivot (adjust until arm “floats”)
    private static final double GRAVITY_FF_NOMINAL = 0.05;     // base when lift down
    private static final double LIFT_SAG_GAIN      = -3e-4;     // extra FF per lift tick

    // –– Motion limits (ticks)
    private static final int MIN_PIVOT = -20, MAX_PIVOT =  600;
    private static final int MIN_LIFT  =    0, MAX_LIFT  = 2200;

    // –– Joystick scaling
    private static final double STICK_PIVOT_SCALAR =  90;   // ticks/sec at full deflection
    private static final double STICK_LIFT_SCALAR  = 250;   // ticks/sec

    /* ----------------------------------------------------------------
       STATE
     ---------------------------------------------------------------- */
    private double targetPivot = 0, targetLift = 0;

    @Override
    public void runOpMode() {

        /* ------------ hardware map ------------ */
        pivotL = hardwareMap.get(DcMotorEx.class, "pivotMotorL");
        pivotR = hardwareMap.get(DcMotorEx.class, "pivotMotorR");
        liftL  = hardwareMap.get(DcMotorEx.class, "liftMotorL");
        liftR  = hardwareMap.get(DcMotorEx.class, "liftMotorR");

        for (DcMotorEx m : new DcMotorEx[]{pivotL,pivotR,liftL,liftR}) {
            m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
        pivotL.setDirection(DcMotorSimple.Direction.REVERSE);
        liftL.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("▶  START\n"
                + "Increment—Pivot: D‑pad ▲/▼  Lift:  X/B\n"
                + "Fine ctrl—Right‑stick‑Y (pivot), Left‑stick‑Y (lift)");
        telemetry.update();
        waitForStart();

        /* ------------ PID bookkeeping ------------ */
        double iPivot = 0, prevErrPivot = 0;
        double iLift  = 0, prevErrLift  = 0;
        ElapsedTime dtTimer = new ElapsedTime();

        while (opModeIsActive()) {

            /* =========================================================
               TARGET UPDATES — bump buttons & fine sticks
             ========================================================= */
            // bump buttons (big-step)
            if (gamepad1.dpad_up)    targetPivot += 10;
            if (gamepad1.dpad_down)  targetPivot -= 10;
            if (gamepad1.x)          targetLift  += 35;
            if (gamepad1.b)          targetLift  -= 35;

            // fine joysticks (proportional rate)
            double dt = dtTimer.seconds();
            dtTimer.reset();

            targetPivot += -gamepad1.right_stick_y * STICK_PIVOT_SCALAR * dt;
            targetLift  += -gamepad1.left_stick_y  * STICK_LIFT_SCALAR  * dt;

            // clamp to safe range
            targetPivot = Range.clip(targetPivot, MIN_PIVOT, MAX_PIVOT);
            targetLift  = Range.clip(targetLift,  MIN_LIFT,  MAX_LIFT);

            /* =========================================================
               PIVOT PID + GRAVITY FEED‑FORWARD
             ========================================================= */
            double posPivot = pivotL.getCurrentPosition();
            double errPivot = targetPivot - posPivot;

            iPivot   += errPivot * dt;
            double dPivot = (errPivot - prevErrPivot) / dt;
            prevErrPivot = errPivot;

            // Pivot angle in radians for cos(θ)         (encoder→deg→rad)
            double thetaRad = posPivot / TICKS_PER_DEG * Math.PI / 180.0;

            double gravityFF = (GRAVITY_FF_NOMINAL + LIFT_SAG_GAIN * liftL.getCurrentPosition()) * Math.cos(thetaRad);

            double outPivot = kP_PIVOT*errPivot + kI_PIVOT*iPivot + kD_PIVOT*dPivot + gravityFF;
            outPivot = Range.clip(outPivot, -1, 1);

            pivotL.setPower(outPivot);
            pivotR.setPower(outPivot);

            /* =========================================================
               LIFT PID
             ========================================================= */
            double posLift = liftL.getCurrentPosition();
            double errLift = targetLift - posLift;

            iLift   += errLift * dt;
            double dLift = (errLift - prevErrLift) / dt;
            prevErrLift = errLift;

            double outLift = kP_LIFT*errLift + kI_LIFT*iLift + kD_LIFT*dLift;
            outLift = Range.clip(outLift, -1, 1);

            liftL.setPower(outLift);
            liftR.setPower(outLift);

            /* =========================================================
               TELEMETRY
             ========================================================= */
            telemetry.addLine("=== Pivot ===");
            telemetry.addData("Tgt",   (int) targetPivot);
            telemetry.addData("Pos",   (int) posPivot);
            telemetry.addData("Err",   (int) errPivot);
            telemetry.addData("FF",    String.format("%.3f", gravityFF));
            telemetry.addData("Pwr",   String.format("%.3f", outPivot));

            telemetry.addLine("=== Lift ===");
            telemetry.addData("Tgt",   (int) targetLift);
            telemetry.addData("Pos",   (int) posLift);
            telemetry.addData("Err",   (int) errLift);
            telemetry.addData("Pwr",   String.format("%.3f", outLift));
            telemetry.update();
        }
    }
}
