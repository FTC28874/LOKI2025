package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="DriveTest", group="Tests")
public class DriveTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor driveFL = null;
    private DcMotor driveFR = null;
    private DcMotor driveBL = null;
    private DcMotor driveBR = null;

    @Override
    public void runOpMode() {

        driveFL  = hardwareMap.get(DcMotor.class, "driveFL");
        driveFR  = hardwareMap.get(DcMotor.class, "driveFR");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");

        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveFR.setDirection(DcMotor.Direction.FORWARD);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveBR.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double powerFL  = axial + lateral + yaw;
            double powerFR = axial - lateral - yaw;
            double powerBL   = axial - lateral + yaw;
            double powerBR  = axial + lateral - yaw;

            max = Math.max(Math.abs(powerFL), Math.abs(powerFR));
            max = Math.max(max, Math.abs(powerBL));
            max = Math.max(max, Math.abs(powerBR));

            if (max > 1.0) {
                powerFL  /= max;
                powerFR /= max;
                powerBL   /= max;
                powerBR  /= max;
            }

            /*
            powerFL  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            powerBL   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            powerFR = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            powerBR  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            driveFL.setPower(powerFL);
            driveFR.setPower(powerFR);
            driveBL.setPower(powerBL);
            driveBR.setPower(powerBR);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", powerFL, powerFR);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", powerBL, powerBR);
            telemetry.update();
        }
    }}
