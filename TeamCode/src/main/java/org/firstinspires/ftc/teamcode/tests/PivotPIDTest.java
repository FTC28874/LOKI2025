package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Pivot PID Test", group = "Tests")
public class PivotPIDTest extends LinearOpMode {

    private DcMotorEx pivotL, pivotR;

    //––– PID coefficients (start with these, then tune) –––
    private static final double kP = 0.008;
    private static final double kI = 0.0000;
    private static final double kD = 0.0003;

    //––– Target position (encoder ticks) –––
    private double targetTicks = 500;   // change on DS or hard‑code

    @Override
    public void runOpMode() {
        // Hardware
        pivotL = hardwareMap.get(DcMotorEx.class, "pivotMotorL");
        pivotR = hardwareMap.get(DcMotorEx.class, "pivotMotorR");

        for (DcMotorEx m : new DcMotorEx[]{pivotL, pivotR}) {
            m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        // Telemetry prompt
        telemetry.addLine("Press ▶ to start, then use ↑/↓ on d‑pad to move target.");
        telemetry.update();
        waitForStart();

        double integral = 0, prevError = 0;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {

            //––– Change target with gamepad –––
            if (gamepad1.dpad_up)    targetTicks += 10;
            if (gamepad1.dpad_down)  targetTicks -= 10;

            //––– Read –––
            double pos = (pivotL.getCurrentPosition() + pivotR.getCurrentPosition()) / 2.0;
            double error = targetTicks - pos;
            double dt = timer.seconds();
            timer.reset();

            //––– PID –––
            integral += error * dt;
            double derivative = (error - prevError) / dt;
            prevError = error;

            double output = kP * error + kI * integral + kD * derivative;
            output = Math.max(-1, Math.min(1, output));   // clip to ±1

            //––– Actuate –––
            pivotL.setPower(output);
            pivotR.setPower(output);

            //––– Telemetry –––
            telemetry.addData("Target (ticks)", targetTicks);
            telemetry.addData("Position", pos);
            telemetry.addData("Error", error);
            telemetry.addData("Power", output);
            telemetry.update();
        }
    }
}