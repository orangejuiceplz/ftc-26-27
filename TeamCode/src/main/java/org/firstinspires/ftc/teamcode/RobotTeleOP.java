// fixed mason bullshit

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Robot TeleOp")
public class RobotTeleOP extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime servoTimer = new ElapsedTime();
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private DcMotor intakeMotor;
    private DcMotor flywheelMotor;
    private Servo servo;

    private static final double stop = 0.05; // mason called this  deadZONE or bs 
    private static final double slowScale = 0.4; // probably not needed but hae r4om last year

    private static final double accelRate = 2.0;
    private static final double decelRaTAE = 4.0;

    private static final double flywheelPower = 0.8;
    private static final double intakePower = 1;

    private static final double servoRangeDegrees = 300;
    private static final double servoRestPosition = 0;
    private static final double servoRotatedPosition = 135 / servoRangeDegrees;
    private static final double servoHoldSeconds = 1.0;
    private static final double triggerThreshold = 0.25;

    @Override
    public void runOpMode() {
        leftWheel = hardwareMap.get(DcMotor.class, "Left");
        rightWheel = hardwareMap.get(DcMotor.class, "Right");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        flywheelMotor = hardwareMap.get(DcMotor.class, "Flywheel");
        servo = hardwareMap.get(Servo.class, "Dual 300 degree");

        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftWheel.setPower(0);
        rightWheel.setPower(0);
        intakeMotor.setPower(0);
        flywheelMotor.setPower(0);
        servo.setPosition(servoRestPosition);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        double leftCurrentPower = 0;
        double rightCurrentPower = 0;
        boolean intakeEnabled = true;
        boolean previousX = false;
        boolean previousTrigger = false;
        boolean servoRotated = false;

        flywheelMotor.setPower(flywheelPower);

        runtime.reset();

        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y;
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

            if (gamepad1.x && !previousX) {
                intakeEnabled = !intakeEnabled;
            }
            previousX = gamepad1.x;
            intakeMotor.setPower(intakeEnabled ? intakePower : 0);

            boolean trigger = gamepad1.right_trigger >= triggerThreshold;
            if (trigger && !previousTrigger && !servoRotated) {
                servoRotated = true;
                servoTimer.reset();
            }
            previousTrigger = trigger;
            if (servoRotated && servoTimer.seconds() >= servoHoldSeconds) {
                servoRotated = false;
            }
            servo.setPosition(servoRotated ? servoRotatedPosition : servoRestPosition);

            telemetry.addData("Intake running", intakeEnabled);
            telemetry.addData("Servo rotated", servoRotated);
            telemetry.addData("Flywheel power", flywheelPower);
            telemetry.update();
        }

        leftWheel.setPower(0);
        rightWheel.setPower(0);
        intakeMotor.setPower(0);
        flywheelMotor.setPower(0);
        servo.setPosition(servoRestPosition);
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
