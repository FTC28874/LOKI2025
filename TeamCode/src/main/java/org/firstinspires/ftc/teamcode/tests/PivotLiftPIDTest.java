package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Pivot + Lift PID Test", group = "Tests")
public class PivotLiftPIDTest extends LinearOpMode {

    //–––– Hardware ––––
    private DcMotorEx pivotL, pivotR;
    private DcMotorEx liftL,  liftR;

    //–––– PID gains (initial guesses) ––––
    // Pivot
    private static final double kP_p = 0.008;
    private static final double kI_p = 0.0000;
    private static final double kD_p = 0.0003;
    // Lift  (vertical screw/chain → usually needs lower kP)
    private static final double kP_l = 0.004;
    private static final double kI_l = 0.0000;
    private static final double kD_l = 0.0002;

    //–––– Targets ––––
    private double targetPivotTicks = 30;
    private double targetLiftTicks  = 0;

    @Override
    public void runOpMode() {

        //–– Map hardware ––
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

        telemetry.addLine("▶ to start\n" +
                "Pivot  : D‑pad ↑/↓\n" +
                "Lift   :  X (up) / B (down)");
        telemetry.update();
        waitForStart();

        //–– Book‑keeping ––
        double intPivot = 0, prevErrPivot = 0;
        double intLift  = 0, prevErrLift  = 0;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {

            //–––– Adjust targets ––––
            if (gamepad1.dpad_up)    targetPivotTicks += 10;
            if (gamepad1.dpad_down)  targetPivotTicks -= 10;
            if (gamepad1.x)          targetLiftTicks  += 20;
            if (gamepad1.b)          targetLiftTicks  -= 20;

            //–––– Timing ––––
            double dt = timer.seconds();
            timer.reset();

            /* =======================================================
               PIVOT PID
             ======================================================= */
            double posPivot = pivotL.getCurrentPosition();                     // assume mirrored encoders
            double errPivot = targetPivotTicks - posPivot;

            intPivot += errPivot * dt;
            double derPivot = (errPivot - prevErrPivot) / dt;
            prevErrPivot = errPivot;

            double outPivot = kP_p*errPivot + kI_p*intPivot + kD_p*derPivot;
            outPivot = Math.max(-1, Math.min(1, outPivot));

            pivotL.setPower(outPivot);
            pivotR.setPower(outPivot);

            /* =======================================================
               LIFT PID
             ======================================================= */
            double posLift = liftL.getCurrentPosition();                       // assume both lifts stay aligned
            double errLift = targetLiftTicks - posLift;

            intLift += errLift * dt;
            double derLift = (errLift - prevErrLift) / dt;
            prevErrLift = errLift;

            double outLift = kP_l*errLift + kI_l*intLift + kD_l*derLift;
            outLift = Math.max(-1, Math.min(1, outLift));

            liftL.setPower(outLift);
            liftR.setPower(outLift);

            //–––– Telemetry ––––
            telemetry.addLine("== Pivot ==");
            telemetry.addData("Target", targetPivotTicks);
            telemetry.addData("Pos",    posPivot);
            telemetry.addData("Err",    errPivot);
            telemetry.addData("Power",  outPivot);
            telemetry.addLine("== Lift ==");
            telemetry.addData("Target", targetLiftTicks);
            telemetry.addData("Pos",    posLift);
            telemetry.addData("Err",    errLift);
            telemetry.addData("Power",  outLift);
            telemetry.update();
        }
    }
}
