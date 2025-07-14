package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.robot.Arm;


@TeleOp(name="ArmTest", group="Tests")
public class ArmTest extends LinearOpMode {

    private int xToggle = 0;
    private int oToggle = 0;
    private int squareToggle = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private Arm arm = null;

    @Override
    public void runOpMode() {

        arm = new Arm(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
//            if (gamepad1.cross) {
//                arm.clawTest();
//            }
//
//            if (gamepad1.circle) {
//                arm.wristTest();
//            }
//
//            if (gamepad1.square) {
//                arm.armTest();
//            }
//
//            if (gamepad1.triangle) {
//                arm.resetServos();
//            }

            if (gamepad1.cross) {
                xToggle = 1;
            } else {
                xToggle = 0;
            }

            if (gamepad1.circle) {
                oToggle = 1;
            } else {
                oToggle = 0;
            }

            if (gamepad1.square) {
                squareToggle = 1;
            } else {
                squareToggle = 0;
            }

            if (xToggle == 1) {
                arm.clawTest();
            } else if (xToggle == 0) {
                arm.clawReset();
            }

            if (oToggle == 1) {
                arm.wristTest();
            } else if (oToggle == 0) {
                arm.wristReset();
            }

            if (squareToggle == 1) {
                arm.armTest();
            } else if (squareToggle == 0) {
                arm.armReset();
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }}
