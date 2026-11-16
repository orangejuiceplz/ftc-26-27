package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Robot TeleOp")
public class RobotTeleOP extends LinearOpMode {
    private static final double deadzone = 0.05;
    private static final double slowScale = 0.4;

    private static final double accelRate = 2.0;
    private static final double decelRate = 4.0;

    private static final double flywheelPower = 0.80;
    private static final double intakePower = 1;

    protected static final double servoRangeDegrees = 300;
    private static final double servoRestPosition = 0;
    private static final double servoHoldSeconds = 1.0;
    private static final double triggerThreshold = 0.25;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime servoTimer = new ElapsedTime();

    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private DcMotor intakeMotor;
    private DcMotor flywheelMotor;
    private Servo servo;

    protected double leftCurrentPower = 0;
    protected double rightCurrentPower = 0;
    private boolean intakeEnabled = true;
    private boolean servoRotated = false;
    private boolean previousX = false;
    private boolean previousCircle = false;
    private boolean previousTrigger = false;

    @Override
    public void runOpMode() {
        announceVersion();
        initHardware();
        stopEverything();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        flywheelMotor.setPower(flywheelPower);
        loopTimer.reset();

        while (opModeIsActive()) {
            updateDrive();
            updateIntake();
            updateServo();
            updateTelemetry();
        }

        stopEverything();
    }

    private void announceVersion() {
        telemetry.addData("On the new version", "yes");
        telemetry.update();
    }

    private void initHardware() {
        leftWheel = hardwareMap.get(DcMotor.class, "Left");
        rightWheel = hardwareMap.get(DcMotor.class, "Right");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        flywheelMotor = hardwareMap.get(DcMotor.class, "Flywheel");
        servo = hardwareMap.get(Servo.class, "Dual 300 degree");

        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void stopEverything() {
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        intakeMotor.setPower(0);
        flywheelMotor.setPower(0);
        servo.setPosition(servoRestPosition);
    }

    private void updateDrive() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        double speedScale = gamepad1.left_bumper ? slowScale : 1;
        double forward = applyDeadzone(gamepad1.left_stick_y) * speedScale;
        double turn = applyDeadzone(-gamepad1.left_stick_x) * speedScale;

        double leftTarget = forward + turn;
        double rightTarget = forward - turn;
        double normalizer = Math.max(1, Math.max(Math.abs(leftTarget), Math.abs(rightTarget)));

        leftCurrentPower = rampUpOrDown(leftCurrentPower, leftTarget / normalizer, dt);
        rightCurrentPower = rampUpOrDown(rightCurrentPower, rightTarget / normalizer, dt);

        leftWheel.setPower(leftCurrentPower);
        rightWheel.setPower(rightCurrentPower);
    }

    private double applyDeadzone(double input) {
        return Math.abs(input) < deadzone ? 0 : input;
    }

    private double rampUpOrDown(double current, double target, double dt) {
        boolean sameDirection = current == 0 || Math.signum(target) == Math.signum(current);
        boolean speedingUp = Math.abs(target) > Math.abs(current) && sameDirection;
        double maxStep = (speedingUp ? accelRate : decelRate) * dt;

        double diff = target - current;
        if (Math.abs(diff) <= maxStep) {
            return target;
        }
        return current + Math.signum(diff) * maxStep;
    }

    private void updateIntake() {
        if (gamepad1.x && !previousX) {
            intakeEnabled = !intakeEnabled;
        }
        previousX = gamepad1.x;

        if (gamepad1.circle && !previousCircle) {
            switchIntakeDirection();
        }
        previousCircle = gamepad1.circle;

        intakeMotor.setPower(intakeEnabled ? intakePower : 0);
    }

    private void switchIntakeDirection() {
        intakeMotor.setDirection(intakeMotor.getDirection().inverted());
    }

    private void updateServo() {
        boolean trigger = gamepad1.right_trigger >= triggerThreshold;
        if (trigger && !previousTrigger && !servoRotated) {
            servoRotated = true;
            servoTimer.reset();
        }
        previousTrigger = trigger;

        if (servoRotated && servoTimer.seconds() >= servoHoldSeconds) {
            servoRotated = false;
        }

        servo.setPosition(servoRotated ? servoRotatedPosition() : servoRestPosition);
    }

    protected double servoRotatedPosition() {
        return 145 / servoRangeDegrees;
    }

    private void updateTelemetry() {
        addTelemetry();
        telemetry.update();
    }

    protected void addTelemetry() {
        telemetry.addData("Intake running", intakeEnabled);
        telemetry.addData("Intake direction", intakeMotor.getDirection());
        telemetry.addData("Servo rotated", servoRotated);
        telemetry.addData("Flywheel power", flywheelPower);
    }
}
