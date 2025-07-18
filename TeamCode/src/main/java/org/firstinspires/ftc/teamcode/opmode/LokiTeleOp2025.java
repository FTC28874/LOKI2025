package org.firstinspires.ftc.teamcode.opmode;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.robot.LokiRobot2025;
import org.firstinspires.ftc.teamcode.common.robot.LiftPivot;
import org.firstinspires.ftc.teamcode.common.robot.Arm;

@TeleOp(name="Loki TeleOp 2025", group="Drive")
public class LokiTeleOp2025 extends OpMode {

    private LokiRobot2025 robot;
    private double wristAdjust = 0;

    private ElapsedTime timer = null;

    private int intakeStage = 0;
    private boolean lastIntakeButton = false;
    private boolean clawClosed = false;
    private boolean xWasPressed = false;

    @Override
    public void init() {
        robot = new LokiRobot2025(hardwareMap);
        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        robot.update();

        // === DRIVER 1 CONTROLS ===
//        double axial   = -gamepad1.left_stick_y + -gamepad2.left_stick_y;
//        double lateral = gamepad1.left_stick_x + gamepad2.left_stick_x;
//        double yaw     = gamepad1.right_stick_x + gamepad2.right_stick_x;

        double axial   = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x * 0.75;
        double yaw     = gamepad1.right_stick_x * 0.5;

        robot.drive.setManualMode();
        robot.drive.drive(axial, lateral, yaw);
        boolean xPressed = gamepad2.x;
        // Claw control
        if (xPressed && !xWasPressed) {
            // Toggle the claw
            clawClosed = !clawClosed;

            if (clawClosed) {
                robot.arm.clawClose();
            } else {
                robot.arm.clawOpen();
            }
        }
        xWasPressed = xPressed;

            if (gamepad2.left_bumper) wristAdjust += 0.02;
        if (gamepad2.right_bumper) wristAdjust -= 0.02;
        wristAdjust = Math.max(0, Math.min(1, wristAdjust));
        robot.arm.setWristServoPosition(robot.arm.wristState.wristServoValue + wristAdjust);

        //climb
        if (gamepad1.back) {
            robot.liftPivot.setPivotState(LiftPivot.PivotState.CLIMB);
            robot.arm.setArmToState(Arm.ArmState.IDLE);
        }
        if (robot.liftPivot.pivotState == LiftPivot.PivotState.CLIMB) {
            if (gamepad1.dpad_up) {
                robot.liftPivot.adjustLiftTarget(35);  // Increase lift height
            } else if (gamepad1.dpad_down) {
                robot.liftPivot.adjustLiftTarget(-35); // Decrease lift height
            }
        }

        //specimen intake
        if (gamepad1.y) {
            robot.liftPivot.setPivotState(LiftPivot.PivotState.SPEC_INTAKE);
            robot.arm.setWristToState(Arm.WristState.HORIZONTAL);
            robot.liftPivot.setLiftState(LiftPivot.LiftState.SPEC_INTAKE);
            robot.arm.setArmToState(Arm.ArmState.SPEC_INTAKE);
        }
        if (gamepad1.x) {
            robot.arm.clawClose();
            timer.reset();
            while (timer.time() < 0.3) {}
            robot.arm.setArmToState(Arm.ArmState.SPEC_SCORE);
            robot.liftPivot.setPivotState(LiftPivot.PivotState.BUCKET);
            timer.reset();
            while (timer.time() < 0.3) {robot.liftPivot.update();}
            robot.liftPivot.setLiftState(LiftPivot.LiftState.SPEC_SCORE);
        }
        if (gamepad1.a) {
            robot.liftPivot.setLiftState(LiftPivot.LiftState.SPEC_HANG);
            timer.reset();
            while (timer.time() < 0.5) {robot.liftPivot.update();}
            robot.arm.clawOpen();
        }

        // === DRIVER 2 CONTROLS ===

        // Presets for intake positions
        boolean intakeButtonPressed = gamepad2.a && !lastIntakeButton;

        if (intakeButtonPressed) {
            if (intakeStage == 0) {
                // Stage 1: move above sample (hover)
                if (robot.liftPivot.isLiftRetractedForPivot()) {
                    robot.liftPivot.setPivotState(LiftPivot.PivotState.INTAKE);
                    robot.arm.setWristToState(Arm.WristState.HORIZONTAL);
                    robot.arm.clawOpen();
                }
                robot.liftPivot.setLiftState(LiftPivot.LiftState.INTAKE_MID);
                robot.arm.setArmToState(Arm.ArmState.IDLE);
                intakeStage = 1;
            } else if (intakeStage == 1) {
                // Stage 2: descend slightly and grab
                robot.liftPivot.setPivotState(LiftPivot.PivotState.LOWERED_INTAKE);
                robot.arm.clawClose();
                intakeStage = 2;
            } else {
                // Reset (optional if you want to do multiple samples)
                intakeStage = 0;
            }
        }
        lastIntakeButton = gamepad2.a;

        if (gamepad2.b) { // IDLE position
            robot.arm.setArmToState(Arm.ArmState.BUCKET);
            if (robot.liftPivot.isLiftRetractedForPivot()) {
                robot.liftPivot.setPivotState(LiftPivot.PivotState.IDLE);
            }
            robot.liftPivot.setLiftState(LiftPivot.LiftState.RETRACTED);
        }

        if (gamepad2.y) { // Scoring BUCKET
            robot.arm.setWristToState(Arm.WristState.VERTICAL);
            robot.liftPivot.setLiftState(LiftPivot.LiftState.BUCKET);
            robot.arm.setArmToState(Arm.ArmState.BUCKET);
            // Delay setting pivot to bucket until lift is up
            if (!robot.liftPivot.isLiftRetractedForPivot()) {
                robot.liftPivot.setPivotState(LiftPivot.PivotState.BUCKET);
            }
        }

        if (robot.liftPivot.pivotState == LiftPivot.PivotState.INTAKE) {
            if (gamepad2.dpad_up) {
                robot.liftPivot.adjustLiftTarget(30);  // Increase lift height
            } else if (gamepad2.dpad_down) {
                robot.liftPivot.adjustLiftTarget(-30); // Decrease lift height
            }
        }

        // Optional: Telemetry for debugging
        telemetry.addData("Lift Pos", robot.liftPivot.getLiftPosition());
        telemetry.addData("Pivot Pos", robot.liftPivot.getPivotPosition());
        telemetry.addData("Wrist Adj", wristAdjust);
        telemetry.update();
    }
}

