package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Robot TeleOp With Turret")
public class TeleOPWithTurret extends RobotTeleOP {
    @Override
    protected double servoRotatedPosition() {
        return 135 / servoRangeDegrees;
    }

    @Override
    protected void addTelemetry() {
        super.addTelemetry();
        telemetry.addData("Left wheel power", leftCurrentPower);
        telemetry.addData("Right wheel power", rightCurrentPower);
    }
}
