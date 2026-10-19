package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp(name = "IliyasKilla", group = "TeleOp")
public class IliyasKilla extends LinearOpMode {
    // heading hold
    public static double HeadingKp = 2.0;
    public static double HeadingKi = 0.03;
    public static double HeadingKd = 0.4;

    public static double HeadingSign = 1.0;

    public static double HeadingMax = 0.5;
    public static double HeadingTolerance = 2.0;

    public static double TurnSettle = 0.3;

    public static double SettledRate = 0;

    public static double StepDegrees = 90;

    // position hold
    public static double XKp = 0.025;
    public static double XKi = 0.0;
    public static double XKd = 0.005;

    public static double YKp = 0.025;
    public static double YKi = 0.0;
    public static double YKd = 0.005;

    public static double PositionMax = 0.0;

    // driving
    public static double StrafeSign = -1.0;

    public static double DriveDeadzone = 0.1;
    public static double DpadSpeed = 0.4;
    public static double HoldDelay = 0.15;
    public static double SlowScale = 0.35;
    public static double TurnScale = 1.0;
    public static double TurnCurve = 1.0;

    public static boolean SmoothSticks = false;

    public static boolean FieldCentric = true;

    // battery
    public static boolean VoltageComp = false;
    public static double NominalVoltage = 12.5;

    // hardware
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor RL;
    private DcMotor RR;

    private GoBildaPinpointDriver Pinpoint;
    private VoltageSensor Battery;

    // state
    private final Pid XPid = new Pid();
    private final Pid YPid = new Pid();
    private final Pid HeadingPid = new Pid();

    private double TargetX;
    private double TargetY;
    private double TargetHeading;

    private final ElapsedTime IdleTimer = new ElapsedTime();
    private ElapsedTime TurnTimer = new ElapsedTime();
    private boolean HoldActive;

    private long LastTime;

    @Override
    public void runOpMode() {
        // setup
        FL = hardwareMap.get(DcMotor.class, "fl");
        FR = hardwareMap.get(DcMotor.class, "fr");
        RL = hardwareMap.get(DcMotor.class, "rl");
        RR = hardwareMap.get(DcMotor.class, "rr");

        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        Battery = hardwareMap.voltageSensor.iterator().next();

        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        RL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor Motor : new DcMotor[]{FL, FR, RL, RR}) {
            Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        Pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Pinpoint.setOffsets(-112.06122589111327, 137.5010986328125, DistanceUnit.MM);
        Pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        Pinpoint.resetPosAndIMU();

        // dashboard
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25);

        FtcDashboard.getInstance().addAction("Reset heading", "Takes the way the robot faces now as forward for driving. Does not move it.", this::ResetHeading);
        FtcDashboard.getInstance().addAction("Hold position here", "Holds the robot on the spot it stands on now. Only pushes back while PositionMax is above 0.", this::StartHold);
        FtcDashboard.getInstance().addAction("Heading step", "Turns the robot by StepDegrees and holds it there, to see how well turning settles. Moves the robot.", this::StepHeading);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        Pinpoint.update();
        TargetHeading = Pinpoint.getHeading(AngleUnit.RADIANS);
        StartHold();

        while (opModeIsActive()) {
            Pinpoint.update();

            double X = Pinpoint.getPosX(DistanceUnit.INCH);
            double Y = Pinpoint.getPosY(DistanceUnit.INCH);
            double Heading = Pinpoint.getHeading(AngleUnit.RADIANS);

            if (gamepad1.options) {
                ResetHeading();
                Heading = 0;
            }

            long Now = System.nanoTime();
            double Dt = (Now - LastTime) / 1e9;
            double LoopMs = Dt * 1000;

            LastTime = Now;

            if (Dt < 0.005 || Dt > 0.05) Dt = 0.02;

            // sticks
            double DriveX;
            double DriveY;

            if (SmoothSticks) {
                double StickX = gamepad1.left_stick_x;
                double StickY = -gamepad1.left_stick_y;
                double Push = Math.min(Math.hypot(StickX, StickY), 1);
                double Scale = Push < DriveDeadzone ? 0 : (Push - DriveDeadzone) / (1 - DriveDeadzone) / Push;

                DriveX = StickX * Scale;
                DriveY = StickY * Scale;
            } else {
                DriveX = Deadzone(gamepad1.left_stick_x);
                DriveY = Deadzone(-gamepad1.left_stick_y);
            }

            double Turn = Deadzone(gamepad1.right_stick_x);

            if (SmoothSticks && Turn != 0) Turn = Math.signum(Turn) * (Math.abs(Turn) - DriveDeadzone) / (1 - DriveDeadzone);

            Turn = Math.signum(Turn) * Math.pow(Math.abs(Turn), TurnCurve) * TurnScale;

            boolean Slow = gamepad1.right_bumper;

            if (Slow) {
                DriveX *= SlowScale;
                DriveY *= SlowScale;
                Turn *= SlowScale;
            }

            double DpadX = ((gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0)) * DpadSpeed;
            double DpadY = ((gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0)) * DpadSpeed;

            // holds
            if (DriveX != 0 || DriveY != 0 || Turn != 0 || DpadX != 0 || DpadY != 0) {
                IdleTimer.reset();
                HoldActive = false;

                XPid.Integral = 0;
                YPid.Integral = 0;
                HeadingPid.Integral = 0;
            } else if (!HoldActive && IdleTimer.seconds() >= HoldDelay) {
                StartHold();
            }

            if (Turn != 0) TurnTimer.reset();

            double TurnRate = Math.abs(Pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            boolean Settled = TurnTimer.seconds() >= TurnSettle || (SettledRate > 0 && Turn == 0 && TurnTimer.seconds() >= 0.1 && TurnRate < SettledRate);

            if (Settled && TurnTimer.seconds() < TurnSettle) {
                TurnTimer = new ElapsedTime(System.nanoTime() - (long) (TurnSettle * 1e9));
            }

            if (TurnTimer.seconds() < TurnSettle) {
                TargetHeading = Heading;
                HeadingPid.Reset();
            }

            double CorrectionForward = 0;
            double CorrectionLeft = 0;

            if (HoldActive) {
                CorrectionForward = Range.clip(XPid.Update(TargetX - X, Dt, XKp, XKi, XKd, 1.0), -PositionMax, PositionMax);
                CorrectionLeft = Range.clip(YPid.Update(TargetY - Y, Dt, YKp, YKi, YKd, 1.0), -PositionMax, PositionMax);
            }

            double Rotation = Turn;

            if (TurnTimer.seconds() >= TurnSettle) {
                double HeadingError = AngleUnit.normalizeRadians(TargetHeading - Heading);
                double Output = HeadingPid.Update(HeadingError, Dt, HeadingKp, HeadingKi, HeadingKd, 0.25);

                if (Math.abs(Math.toDegrees(HeadingError)) < HeadingTolerance) {
                    HeadingPid.Integral = 0;
                    Output = 0;
                }

                Rotation = Range.clip(-HeadingSign * Output, -HeadingMax, HeadingMax);
            }

            // wheels
            double FieldX = DriveX - CorrectionLeft;
            double FieldY = DriveY + CorrectionForward;

            double Magnitude = Math.hypot(FieldX, FieldY);

            if (Magnitude > 1) {
                FieldX /= Magnitude;
                FieldY /= Magnitude;
            }

            double Facing = FieldCentric ? Heading : 0;
            double Cos = Math.cos(-Facing);
            double Sin = Math.sin(-Facing);

            double RobotX = (FieldX * Cos - FieldY * Sin + DpadX) * StrafeSign;
            double RobotY = FieldX * Sin + FieldY * Cos + DpadY;

            double Denominator = Math.max(Math.abs(RobotX) + Math.abs(RobotY) + Math.abs(Rotation), 1);

            double PowerFL = (RobotY + RobotX + Rotation) / Denominator;
            double PowerRL = (RobotY - RobotX + Rotation) / Denominator;
            double PowerFR = (RobotY - RobotX - Rotation) / Denominator;
            double PowerRR = (RobotY + RobotX - Rotation) / Denominator;

            double Volts = VoltageComp ? Battery.getVoltage() : 0;

            if (VoltageComp && Volts > 6) {
                double Boost = NominalVoltage / Volts;

                PowerFL = Range.clip(PowerFL * Boost, -1, 1);
                PowerRL = Range.clip(PowerRL * Boost, -1, 1);
                PowerFR = Range.clip(PowerFR * Boost, -1, 1);
                PowerRR = Range.clip(PowerRR * Boost, -1, 1);
            }

            FL.setPower(PowerFL);
            RL.setPower(PowerRL);
            FR.setPower(PowerFR);
            RR.setPower(PowerRR);

            // telemetry
            telemetry.addData("Pose", "%.2f, %.2f, %.1f", X, Y, Math.toDegrees(Heading));
            telemetry.addData("Target", "%.2f, %.2f, %.1f", TargetX, TargetY, Math.toDegrees(TargetHeading));
            telemetry.addData("Error", "%.2f, %.2f, %.1f", TargetX - X, TargetY - Y, Math.toDegrees(AngleUnit.normalizeRadians(TargetHeading - Heading)));

            telemetry.addData("Mode", (HoldActive ? "Hold" : "Driver") + (Slow ? ", slow" : "") + (FieldCentric ? "" : ", robot centric"));
            telemetry.addData("Loop", "%.1f ms", LoopMs);
            telemetry.addData("Correction", "fwd %.3f  left %.3f  rot %.3f", CorrectionForward, CorrectionLeft, Rotation);
            telemetry.addData("Pinpoint", Pinpoint.getDeviceStatus());

            telemetry.update();

            TelemetryPacket Packet = new TelemetryPacket();

            Packet.put("X", X);
            Packet.put("Y", Y);
            Packet.put("Heading", Math.toDegrees(Heading));

            Packet.put("Target X", TargetX);
            Packet.put("Target Y", TargetY);
            Packet.put("Target Heading", Math.toDegrees(TargetHeading));

            Packet.put("Correction Forward", CorrectionForward);
            Packet.put("Correction Left", CorrectionLeft);
            Packet.put("Rotation", Rotation);

            Packet.put("Drive X", DriveX);
            Packet.put("Drive Y", DriveY);
            Packet.put("Turn", Turn);
            Packet.put("Turn Rate", TurnRate);
            Packet.put("Slow", Slow ? 1 : 0);
            Packet.put("Loop ms", LoopMs);
            if (VoltageComp) Packet.put("Battery", Volts);

            Packet.put("FL", PowerFL);
            Packet.put("FR", PowerFR);
            Packet.put("RL", PowerRL);
            Packet.put("RR", PowerRR);

            Packet.fieldOverlay()
                    .setStroke("#3F51B5")
                    .strokeCircle(X, Y, 9)
                    .strokeLine(X, Y, X + 9 * Math.cos(Heading), Y + 9 * Math.sin(Heading))
                    .setStroke("#4CAF50")
                    .strokeCircle(TargetX, TargetY, 2);

            FtcDashboard.getInstance().sendTelemetryPacket(Packet);
        }
    }

    private void StartHold() {
        TargetX = Pinpoint.getPosX(DistanceUnit.INCH);
        TargetY = Pinpoint.getPosY(DistanceUnit.INCH);

        XPid.Reset();
        YPid.Reset();

        LastTime = System.nanoTime();
        HoldActive = true;
    }

    private void ResetHeading() {
        Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, Pinpoint.getPosX(DistanceUnit.INCH), Pinpoint.getPosY(DistanceUnit.INCH), AngleUnit.RADIANS, 0));

        TargetHeading = 0;
        HeadingPid.Reset();
    }

    private void StepHeading() {
        TargetHeading = AngleUnit.normalizeRadians(TargetHeading + Math.toRadians(StepDegrees));
        HeadingPid.Reset();

        TurnTimer = new ElapsedTime(System.nanoTime() - (long) (TurnSettle * 1e9));
    }

    private static double Deadzone(double Value) {
        return Math.abs(Value) < DriveDeadzone ? 0 : Value;
    }

    private static class Pid {
        double Integral;
        double LastError;

        double Update(double Error, double Dt, double Kp, double Ki, double Kd, double IntegralLimit) {
            Integral = Range.clip(Integral + Error * Dt, -IntegralLimit, IntegralLimit);

            double Derivative = (Error - LastError) / Dt;
            LastError = Error;

            return Kp * Error + Ki * Integral + Kd * Derivative;
        }

        void Reset() {
            Integral = 0;
            LastError = 0;
        }
    }
}
