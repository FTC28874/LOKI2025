package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.robot.LokiRobot2025;
import org.firstinspires.ftc.teamcode.common.robot.LiftPivot;
import org.firstinspires.ftc.teamcode.common.robot.Arm;

@TeleOp(name="Loki TeleOp 2025", group="Drive")
public class LokiTeleOp2025 extends OpMode {

    private LokiRobot2025 robot;
    private double wristAdjust = 0;

    @Override
    public void init() {
        robot = new LokiRobot2025(hardwareMap);
    }

    @Override
    public void loop() {
        robot.update();

        // === DRIVER 1 CONTROLS ===
        double axial   = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw     = gamepad1.right_stick_x;

        robot.drive.drive(axial, lateral, yaw);

        // Claw control
        if (gamepad1.right_bumper) robot.arm.clawClose();
        if (gamepad1.left_bumper) robot.arm.clawOpen();

        // Fine wrist adjustment
        if (gamepad1.dpad_up) wristAdjust += 0.005;
        if (gamepad1.dpad_down) wristAdjust -= 0.005;
        wristAdjust = Math.max(0, Math.min(1, wristAdjust));
        robot.arm.setWristServoPosition(robot.arm.wristState.wristServoValue + wristAdjust);


        // === DRIVER 2 CONTROLS ===

        // Presets for intake positions
        if (gamepad2.a) {
            robot.liftPivot.setLiftState(LiftPivot.LiftState.RETRACTED);
            if (robot.liftPivot.isLiftRetractedForPivot()) {
                // Go slightly below normal intake to secure object
                robot.liftPivot.setPivotState(LiftPivot.PivotState.INTAKE);
            }
        }

        // Close claw automatically once pivot is near intake position
        if (robot.liftPivot.pivotState == LiftPivot.PivotState.INTAKE &&
                Math.abs(robot.liftPivot.getPivotPosition() - LiftPivot.PivotState.INTAKE.encoderValue) < 10 &&
                gamepad2.a) {
            robot.arm.clawClose();
        }

        if (gamepad2.b) { // IDLE position
            if (robot.liftPivot.isLiftRetractedForPivot()) {
                robot.liftPivot.setPivotState(LiftPivot.PivotState.IDLE);
            }
            robot.liftPivot.setLiftState(LiftPivot.LiftState.RETRACTED);
        }

        if (gamepad2.y) { // Scoring BUCKET
            robot.liftPivot.setLiftState(LiftPivot.LiftState.BUCKET);
            robot.arm.setArmToState(Arm.ArmState.BUCKET);
            // Delay setting pivot to bucket until lift is up
            if (!robot.liftPivot.isLiftRetractedForPivot()) {
                robot.liftPivot.setPivotState(LiftPivot.PivotState.BUCKET);
            }
        }

        // Wrist override (Driver 2)
        if (gamepad2.dpad_up) wristAdjust += 0.005;
        if (gamepad2.dpad_down) wristAdjust -= 0.005;
        wristAdjust = Math.max(0, Math.min(1, wristAdjust));
        robot.arm.setWristServoPosition(robot.arm.wristState.wristServoValue + wristAdjust);

        // Optional: Telemetry for debugging
        telemetry.addData("Lift Pos", robot.liftPivot.getLiftPosition());
        telemetry.addData("Pivot Pos", robot.liftPivot.getPivotPosition());
        telemetry.addData("Wrist Adj", wristAdjust);
        telemetry.update();
    }
}

