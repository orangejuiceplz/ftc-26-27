package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Robot TeleOp With Turret")
public class TeleOPWithTurret extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private DcMotor intakeMotor;
    private DcMotor flywheelMotor;
    private Servo servo;

    private static final double stop = 0.05;
    private static final double slowScale = 0.4;
    private static final double accelRate = 2.0;
    private static final double decelRate = 4.0;

    private static final double servoRangeDegrees = 300;
    private static final double servoRestPosition = 0;
    private static final double servoRotatedPosition = 135 / servoRangeDegrees;
    private static final double triggerThreshold = 0.25;
    private static final double flywheelPower = 0       ;
    private static final double intakePower = 1;

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

        telemetry.addData("Init", "Robot initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        flywheelMotor.setPower(flywheelPower);

        double leftCurrentPower = 0;
        double rightCurrentPower = 0;
        boolean intakeEnabled = true;
        boolean previousX = false;

        runtime.reset();

        while (opModeIsActive()) {
            double dt = runtime.seconds();
            runtime.reset();

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

            boolean servoRotated = gamepad1.right_trigger >= triggerThreshold;
            servo.setPosition(servoRotated ? servoRotatedPosition : servoRestPosition);

            telemetry.addData("Intake running", intakeEnabled);
            telemetry.addData("Servo rotated", servoRotated);
            telemetry.addData("Left wheel power", leftCurrentPower);
            telemetry.addData("Right wheel power", rightCurrentPower);
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
        double rate = speedingUp ? accelRate : decelRate;
        double maxStep = rate * dt;

        double diff = target - current;
        if (Math.abs(diff) <= maxStep) {
            return target;
        }
        return current + Math.signum(diff) * maxStep;
    }
}
