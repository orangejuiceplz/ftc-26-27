// fixed mason bullshit

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Robot TeleOp")
public class RobotTeleOP extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftWheel;
    private DcMotor rightWheel;

    private static final double stop = 0.05; // mason called this  deadZONE or bs 
    private static final double slowScale = 0.4; // probably not needed but hae r4om last year

    private static final double accelRate = 2.0;
    private static final double decelRaTAE = 4.0;

    @Override
    public void runOpMode() {
        leftWheel = hardwareMap.get(DcMotor.class, "leftWheel");
        rightWheel = hardwareMap.get(DcMotor.class, "rightWheel");

        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftWheel.setPower(0);
        rightWheel.setPower(0);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        double leftCurrentPower = 0;
        double rightCurrentPower = 0;

        runtime.reset();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;

            if (Math.abs(forward) < stop) {
                forward = 0;
            }
            if (Math.abs(turn) < stop) {
                turn = 0;
            }

            if (gamepad1.left_bumper) {
                forward *= slowScale;
                turn *= slowScale;
            }

            double dt = runtime.seconds();
            runtime.reset();

            double leftPower = forward + turn;
            double rightPower = forward - turn;
            double scale = Math.max(1, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
            leftPower = leftPower / scale;
            rightPower = rightPower / scale;

            leftCurrentPower = rampUpOrDown(leftCurrentPower, leftPower, dt);
            rightCurrentPower = rampUpOrDown(rightCurrentPower, rightPower, dt);

            leftWheel.setPower(leftCurrentPower);
            rightWheel.setPower(rightCurrentPower);

        }

        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }

    private double rampUpOrDown(double current, double target, double dt) {
        boolean speedingUp = Math.abs(target) > Math.abs(current) && (current == 0 || Math.signum(target) == Math.signum(current));
        double rate = speedingUp ? accelRate : decelRaTAE;
        double maxStep = rate * dt;

        double diff = target - current;
        if (Math.abs(diff) <= maxStep) {
            return target;
        }
        return current + Math.signum(diff) * maxStep;
    }
}
