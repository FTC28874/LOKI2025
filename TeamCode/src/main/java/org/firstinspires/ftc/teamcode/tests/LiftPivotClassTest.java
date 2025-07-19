package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.robot.LiftPivot;

@TeleOp(name="LiftPivotClassTest", group="Tests")
public class LiftPivotClassTest extends LinearOpMode {
    LiftPivot liftPivot;

    @Override
    public void runOpMode() {
        liftPivot = new LiftPivot(hardwareMap, true);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) liftPivot.setLiftState(LiftPivot.LiftState.BUCKET);
            if (gamepad1.b) liftPivot.setLiftState(LiftPivot.LiftState.RETRACTED);
            if (gamepad1.dpad_up) liftPivot.setPivotState(LiftPivot.PivotState.BUCKET);
            if (gamepad1.dpad_down) liftPivot.setPivotState(LiftPivot.PivotState.INTAKE);

            liftPivot.update();  // ← PID control + FF

            telemetry.addData("Pivot Pos", liftPivot.getPivotPosition());
            telemetry.addData("Lift Pos", liftPivot.getLiftPosition());
            telemetry.update();
        }
    }

}
