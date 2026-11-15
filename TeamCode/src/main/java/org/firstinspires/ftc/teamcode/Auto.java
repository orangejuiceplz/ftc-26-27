package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auto_Drive_2ft")
public class Auto extends LinearOpMode {

    private DcMotor motor_right;
    private DcMotor motor_left;

    static final double TICKS_PER_REV = 537.7;
    static final double WHEEL_DIAMETER_IN = 3.77953;
    static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_IN * Math.PI);
    static final double power = 0.5;

    @Override
    public void runOpMode() {
        motor_right = hardwareMap.get(DcMotor.class, "Right");
        motor_left = hardwareMap.get(DcMotor.class, "Left");

        motor_left.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized. Ticks/Inch: %.2f", TICKS_PER_INCH);
        telemetry.update();

        waitForStart();

        driveInches(12, power);
    }

    public void driveInches(double inches, double power) {
        int moveTicks = (int) (inches * TICKS_PER_INCH);
        int leftTarget = motor_left.getCurrentPosition() + moveTicks;
        int rightTarget = motor_right.getCurrentPosition() + moveTicks;

        motor_left.setTargetPosition(leftTarget);
        motor_right.setTargetPosition(rightTarget);

        motor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor_left.setPower(power);
        motor_right.setPower(power);

        while (opModeIsActive() && motor_left.isBusy() && motor_right.isBusy()) {
            telemetry.addData("Left Pos", motor_left.getCurrentPosition());
            telemetry.addData("Right Pos", motor_right.getCurrentPosition());
            telemetry.addData("Target", leftTarget);
            telemetry.update();
        }

        motor_left.setPower(0);
        motor_right.setPower(0);
        motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
