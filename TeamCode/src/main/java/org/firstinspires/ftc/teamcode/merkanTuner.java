package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.swervely.PinpointSetup;

/**
 * Hold-controller TeleOp (your merkanTuner, fixed).
 *
 * Frames used in this file:
 *   Driver / field frame : x = RIGHT, y = FORWARD   (heading 0 = the way the robot faced at reset)
 *   Pinpoint frame       : x = FORWARD, y = LEFT, heading counter-clockwise positive
 *   Mecanum mix          : rotationPower positive = CLOCKWISE
 *
 * Controls: left stick = drive, right stick X = turn.
 *   X (cross)    = set the way the robot faces now as "forward"
 *   OPTIONS      = full Pinpoint IMU reset (keep the robot still)
 *   RIGHT BUMPER = robot-centric driving (no field-centric, no hold) for testing
 */
@TeleOp(name = "tuner", group = "TeleOp")
public class merkanTuner extends LinearOpMode {

    // ---------- MOTOR DIRECTIONS (set these with WheelTest) ----------
    // Preset = your Pedro drivetrainConfig. Positive power must spin each wheel toward the robot's front.
    static final DcMotorSimple.Direction FL_DIR = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction FR_DIR = DcMotorSimple.Direction.REVERSE;
    static final DcMotorSimple.Direction RL_DIR = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction RR_DIR = DcMotorSimple.Direction.REVERSE;

    // ---------- TURN THIS ON AFTER DRIVING WORKS ----------
    public static boolean HOLD_ENABLED = false;

    // HEADING
    public static double HEADING_KP = 0.75;
    public static double HEADING_KI = 0.03;
    public static double HEADING_KD = 0.4;
    // FIX: the mix turns CLOCKWISE for positive power, but Pinpoint heading grows COUNTER-clockwise,
    // so the correction must be negated. (+1 here made the robot push the wrong way / spin.)
    public static double HEADING_SIGN = -1.0;
    public static double HEADING_MAX = 0.5;
    public static double HEADING_TOLERANCE = 2.0;

    // POSITION
    public static double X_KP = 0.025, X_KI = 0.0, X_KD = 0.005;
    public static double Y_KP = 0.025, Y_KI = 0.0, Y_KD = 0.005;
    public static double POSITION_MAX = 0.20;

    // DRIVER
    public static double DRIVE_DEADZONE = 0.06;
    public static double HOLD_DELAY = 0.15;

    private DcMotor frontLeft, frontRight, rearLeft, rearRight;
    private GoBildaPinpointDriver pinpoint;

    private double targetX, targetY, targetHeading;
    private double headingIntegral, headingLastError;
    private double xIntegral, xLastError, yIntegral, yLastError;
    private long lastTime;
    private boolean holdActive = true;
    private long driverLastActiveTime;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        rearLeft = hardwareMap.get(DcMotor.class, "rl");
        rearRight = hardwareMap.get(DcMotor.class, "rr");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        frontLeft.setDirection(FL_DIR);
        frontRight.setDirection(FR_DIR);
        rearLeft.setDirection(RL_DIR);
        rearRight.setDirection(RR_DIR);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, rearLeft, rearRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // pod type, offsets and pod directions come from PinpointSetup, shared with every program
        PinpointSetup.apply(pinpoint);
        pinpoint.resetPosAndIMU();

        telemetry.addLine("TESTSETS3 ready. Hold starts " + (HOLD_ENABLED ? "ENABLED" : "DISABLED"));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        pinpoint.update();
        targetX = pinpoint.getPosX(DistanceUnit.INCH);
        targetY = pinpoint.getPosY(DistanceUnit.INCH);
        targetHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        resetControllers();
        holdActive = true;
        driverLastActiveTime = System.currentTimeMillis();

        boolean lastCross = false, lastOptions = false;
        long imuResetStart = 0;
        boolean imuResetting = false;

        while (opModeIsActive()) {
            pinpoint.update();

            double currentX = pinpoint.getPosX(DistanceUnit.INCH);   // forward
            double currentY = pinpoint.getPosY(DistanceUnit.INCH);   // left
            double currentHeading = pinpoint.getHeading(AngleUnit.RADIANS);

            // ---- IMU reset (OPTIONS) ----
            if (gamepad1.options && !lastOptions && !imuResetting) {
                pinpoint.resetPosAndIMU();
                imuResetting = true;
                imuResetStart = System.currentTimeMillis();
            }
            lastOptions = gamepad1.options;
            if (imuResetting) {
                setAllPower(0, 0, 0, 0);
                if (System.currentTimeMillis() - imuResetStart > 500) {
                    pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
                    targetX = 0; targetY = 0; targetHeading = 0;
                    resetControllers();
                    holdActive = true;
                    imuResetting = false;
                }
                telemetry.addLine("Resetting IMU... keep the robot still");
                telemetry.update();
                continue;
            }

            // ---- re-zero "forward" (X / cross) ----
            if (gamepad1.cross && !lastCross) {
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, currentX, currentY, AngleUnit.DEGREES, 0));
                currentHeading = 0;
                targetHeading = 0;
                resetControllers();
            }
            lastCross = gamepad1.cross;

            long now = System.nanoTime();
            double dt = (now - lastTime) / 1_000_000_000.0;
            lastTime = now;
            if (dt < 0.005 || dt > 0.05) dt = 0.02;

            // ---- sticks ----
            double driveX = gamepad1.left_stick_x;     // right +
            double driveY = -gamepad1.left_stick_y;    // FIX: stick up is negative, so negate -> forward +
            double turn = gamepad1.right_stick_x;      // right + = clockwise

            if (Math.abs(driveX) < DRIVE_DEADZONE) driveX = 0;
            if (Math.abs(driveY) < DRIVE_DEADZONE) driveY = 0;
            if (Math.abs(turn) < DRIVE_DEADZONE) turn = 0;

            boolean driverActive = driveX != 0 || driveY != 0 || turn != 0;

            if (driverActive) {
                driverLastActiveTime = System.currentTimeMillis();
                holdActive = false;
                xIntegral = 0; yIntegral = 0; headingIntegral = 0;
                if (turn != 0) {
                    targetHeading = currentHeading;
                    headingLastError = 0;
                }
            }

            if (!driverActive && !holdActive
                    && System.currentTimeMillis() - driverLastActiveTime >= HOLD_DELAY * 1000.0) {
                targetX = currentX;
                targetY = currentY;
                targetHeading = currentHeading;
                resetControllers();
                holdActive = true;
            }

            // ---- position hold ----
            // FIX: errors are in the Pinpoint frame (x forward, y left). Convert to driver frame:
            //   right = -pinpointY, forward = pinpointX
            double correctionForward = 0, correctionRight = 0;
            if (HOLD_ENABLED && holdActive) {
                double errForward = targetX - currentX;
                double errRight = -(targetY - currentY);

                xIntegral = clip(xIntegral + errForward * dt, -1, 1);
                double xDer = (errForward - xLastError) / dt;
                xLastError = errForward;
                correctionForward = clip(X_KP * errForward + X_KI * xIntegral + X_KD * xDer,
                        -POSITION_MAX, POSITION_MAX);

                yIntegral = clip(yIntegral + errRight * dt, -1, 1);
                double yDer = (errRight - yLastError) / dt;
                yLastError = errRight;
                correctionRight = clip(Y_KP * errRight + Y_KI * yIntegral + Y_KD * yDer,
                        -POSITION_MAX, POSITION_MAX);
            }

            // ---- heading ----
            double rotationPower;
            if (turn != 0 || !HOLD_ENABLED) {
                rotationPower = turn;
            } else {
                double headingError = angleWrap(targetHeading - currentHeading);   // + = need CCW
                if (Math.abs(Math.toDegrees(headingError)) < HEADING_TOLERANCE) headingError = 0;

                headingIntegral = clip(headingIntegral + headingError * dt, -0.25, 0.25);
                double der = (headingError - headingLastError) / dt;
                headingLastError = headingError;

                rotationPower = (HEADING_KP * headingError + HEADING_KI * headingIntegral + HEADING_KD * der)
                        * HEADING_SIGN;
                rotationPower = clip(rotationPower, -HEADING_MAX, HEADING_MAX);
            }

            // ---- driver/field -> robot frame ----
            double robotX, robotY;
            if (gamepad1.right_bumper) {
                robotX = driveX;           // robot-centric test mode
                robotY = driveY;
            } else {
                double fieldX = driveX + correctionRight;
                double fieldY = driveY + correctionForward;
                double mag = Math.hypot(fieldX, fieldY);
                if (mag > 1.0) { fieldX /= mag; fieldY /= mag; }

                double cos = Math.cos(-currentHeading);
                double sin = Math.sin(-currentHeading);
                robotX = fieldX * cos - fieldY * sin;
                robotY = fieldX * sin + fieldY * cos;
            }

            // the wheels are on in the other pattern; IliyasKilla.StrafeSign says which way sliding goes
            robotX *= IliyasKilla.StrafeSign;

            // ---- mecanum (rotation + = clockwise) ----
            double denom = Math.max(Math.abs(robotY) + Math.abs(robotX) + Math.abs(rotationPower), 1.0);
            double fl = (robotY + robotX + rotationPower) / denom;
            double rl = (robotY - robotX + rotationPower) / denom;
            double fr = (robotY - robotX - rotationPower) / denom;
            double rr = (robotY + robotX - rotationPower) / denom;
            setAllPower(fl, fr, rl, rr);

            telemetry.addData("mode", gamepad1.right_bumper ? "ROBOT-CENTRIC" : "FIELD-CENTRIC");
            telemetry.addData("hold", HOLD_ENABLED ? (holdActive ? "ACTIVE" : "DRIVER") : "DISABLED");
            telemetry.addData("pos fwd/left (in)", "%.1f  %.1f", currentX, currentY);
            telemetry.addData("heading / target", "%.1f  %.1f", Math.toDegrees(currentHeading), Math.toDegrees(targetHeading));
            telemetry.addData("sticks x/y/turn", "%.2f %.2f %.2f", driveX, driveY, turn);
            telemetry.addData("wheel fl fr rl rr", "%.2f %.2f %.2f %.2f", fl, fr, rl, rr);
            telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
            telemetry.update();
        }
        setAllPower(0, 0, 0, 0);
    }

    private void setAllPower(double fl, double fr, double rl, double rr) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        rearLeft.setPower(rl);
        rearRight.setPower(rr);
    }

    private void resetControllers() {
        headingIntegral = 0; headingLastError = 0;
        xIntegral = 0; xLastError = 0;
        yIntegral = 0; yLastError = 0;
        lastTime = System.nanoTime();
    }

    private double angleWrap(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}