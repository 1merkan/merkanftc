package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IliyasKilla;
import org.firstinspires.ftc.teamcode.swervely.PinpointSetup;

public class Constants {
    /*
     * pedro's mecanum mix assumes the usual wheel pattern and has no strafe setting. this robot's
     * wheels are on the other way round, which IliyasKilla handles with StrafeSign = -1. swapping
     * the front and back names on each side flips only the strafe (both wheels on a side spin the
     * same way for forward and turning), so pedro slides the same way IliyasKilla does. it reads
     * StrafeSign, so the drivers' setting is the only one to change if the wheels are ever swapped
     */
    public static MecanumConfig drivetrainConfig() {
        boolean Flipped = IliyasKilla.StrafeSign < 0;

        return new MecanumConfig(c -> {
            c.frontLeftName.set(Flipped ? "rl" : "fl");
            c.frontRightName.set(Flipped ? "rr" : "fr");
            c.backLeftName.set(Flipped ? "fl" : "rl");
            c.backRightName.set(Flipped ? "fr" : "rr");
            c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        });
    }

    /*
     * the pinpoint settings come from swervely/PinpointSetup, so pedro, IliyasKilla and merkanTuner
     * all agree about the pods. this is a method and not a static field on purpose: a field gets
     * built once when the app loads, so a change made live on the dashboard would only count after
     * a restart. built here, it counts from the next init.
     *
     * how pedro 3.0.1 hands these to the pinpoint (PinpointLocalizer's constructor, read in the
     * revhub 3.0.1 sources):
     *   setOffsets(xPodOffset, yPodOffset, offsetUnits)
     *   setEncoderDirections(xPodDirection, yPodDirection)
     *   setEncoderResolution(podType)
     * straight through, nothing swapped or negated. so it means exactly what it means in
     * PinpointSetup.apply: xPodOffset is how far sideways the forward pod sits (left positive),
     * yPodOffset how far forward the strafe pod sits (front positive), here in millimetres.
     * the pose pedro reads back is getPosX, getPosY (in globalDistanceUnit) and getHeading, also
     * unchanged, so pedro's x and y are the pinpoint's x and y.
     *
     * the Pinpoint tuner prints a "localizerConfig" block. put its numbers into PinpointSetup
     * instead (its offsets are inches, times 25.4 for millimetres).
     */
    public static PinpointConfig localizerConfig() {
        return new PinpointConfig(c -> {
            c.name.set("pinpoint");
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            c.xPodOffset.set(PinpointSetup.XOffsetMm);
            c.yPodOffset.set(PinpointSetup.YOffsetMm);
            c.xPodDirection.set(PinpointSetup.Direction(PinpointSetup.XPodDirection));
            c.yPodDirection.set(PinpointSetup.Direction(PinpointSetup.YPodDirection));
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.MM);
        });
    }

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.18591782169151252);
                Controller secondaryTranslationalForward = Controller.proportional(0.06869166729932472);
                Controller primaryTranslationalLateral = Controller.proportional(0.30410229581897497);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11235767254211265);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011115948064045797));
                c.brake.set(Controller.proportionalFeedforward(0.009448555854438927));

                c.headingFeedback.set(Controller.proportional(728.5585366900535));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(-0.04262935252820299, 0.076916659918455));

                c.linearBrakeCoefficients.set(Matrix.diag(19.769062389351014, -0.6282632527562289));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-0.28094353773406583, 0.03925858952434869));

                c.maxAchievableForwardVelocity.set(85.75658003424422);
                c.maxAchievableStrafeVelocity.set(57.16565718842405);
                c.naturalForwardDeceleration.set(32.34505947763211);
                c.naturalStrafeDeceleration.set(49.812086747122635);
            }
    );

    /**
     * builds a pedro follower from the parts above, the same three the Foresight tuner uses. the
     * pinpoint settings are read from PinpointSetup right now, so call this at init. it recalibrates
     * the pinpoint's gyro and waits half a second, so the robot has to sit still.
     *
     * @param hardwareMap the OpMode's hardwareMap
     * @return a follower that isnt following anything yet
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(new PinpointLocalizer(hardwareMap, localizerConfig()), new Mecanum(hardwareMap, drivetrainConfig()), new Foresight(foresightConfig));
    }
}