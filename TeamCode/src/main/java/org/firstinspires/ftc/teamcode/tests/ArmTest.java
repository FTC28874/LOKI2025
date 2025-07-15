package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.robot.Arm;


@TeleOp(name="ArmTest", group="Tests")
public class ArmTest extends LinearOpMode {

    private boolean xToggle = false;
    private boolean oToggle = false;
    private boolean squareToggle = false;
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
//hi
//            if (gamepad1.triangle) {
//                arm.resetServos();
//            }

            if (gamepad1.cross && xToggle) {
                xToggle = false;
                sleep(200);
            } else if (gamepad1.cross && !xToggle){
                xToggle = true;
                sleep(200);
            }

            if (gamepad1.circle && oToggle) {
                oToggle = false;
                sleep(200);
            } else if (gamepad1.circle && !oToggle){
                oToggle = true;
                sleep(200);
            }

            if (gamepad1.square && squareToggle) {
                squareToggle = false;
                sleep(200);
            } else if (gamepad1.square && !squareToggle){
                squareToggle = true;
                sleep(200);
            }


            if (xToggle) {
                arm.clawTest();
            } else if (!xToggle) {
                arm.clawReset();
            }

            if (oToggle) {
                arm.wristTest();
            } else if (!oToggle) {
                arm.wristReset();
            }

            if (squareToggle) {
                arm.armTest();
            } else if (!squareToggle) {
                arm.armReset();
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }}
