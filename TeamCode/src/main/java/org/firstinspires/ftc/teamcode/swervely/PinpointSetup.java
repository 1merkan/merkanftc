package org.firstinspires.ftc.teamcode.swervely;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * the one place that says how the pinpoint is mounted on this robot: which way each tracking
 * wheel counts and where each one sits. SwervelyOdometry uses it to measure and check these numbers,
 * and IliyasKilla, merkanTuner and pedro (Constants.createFollower) all read them from here.
 *
 * every "public static" number here shows up on the dashboard as "PinpointSetup" and can be changed
 * live. SwervelyOdometry picks a change up straight away, other programs only read it at init.
 */
@Config
public class PinpointSetup {
    /*
     * which way each tracking wheel counts, "FORWARD" or "REVERSED".
     * the x wheel (the one rolling forward/back) has to count up when the robot drives forward.
     * the y wheel (the one rolling sideways) has to count up when the robot slides LEFT.
     * if one counts the wrong way, flip it. anything that isnt "REVERSED" counts as "FORWARD".
     * REVERSED and REVERSED is what IliyasKilla uses today.
     */
    public static String XPodDirection = "REVERSED";
    public static String YPodDirection = "REVERSED";

    /*
     * where the tracking wheels sit, in millimetres, measured from the centre of the robot (the
     * point the pinpoint reports the position of). this is goBILDA's convention, from the Javadoc
     * of GoBildaPinpointDriver.setOffsets in the FTC SDK 12.0.0:
     *
     *   XOffsetMm  how far SIDEWAYS from the centre the x (forward) wheel is.
     *              left of the centre is positive, right is negative.
     *   YOffsetMm  how far FORWARD from the centre the y (strafe) wheel is.
     *              in front of the centre is positive, behind is negative.
     *
     * note the names cross over: the x offset is a sideways distance and the y offset is a
     * forward distance. getting them swapped or with the wrong sign makes the position wander off
     * when the robot spins in place, which is the easiest way to spot it.
     *
     * these are IliyasKilla's -4.411859287051704 and 5.413429080031989 inches times 25.4.
     */
    public static double XOffsetMm = -112.06122589111327;
    public static double YOffsetMm = 137.5010986328125;

    /*
     * how many encoder ticks the goBILDA 4-bar pod counts per millimetre it rolls. the driver keeps
     * this number private, so it is copied here (GoBildaPinpointDriver.goBILDA_4_BAR_POD).
     * final, so it doesnt show up on the dashboard: it is a fact about the pods, not a setting.
     */
    public static final double TicksPerMm = 19.89436789;

    /**
     * sends the pod type, the offsets and the directions above to the pinpoint. it doesnt reset
     * the position, so it is safe to call again while the robot is being tracked.
     *
     * @param Pinpoint the pinpoint to set up
     */
    public static void apply(GoBildaPinpointDriver Pinpoint) {
        Pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Pinpoint.setOffsets(XOffsetMm, YOffsetMm, DistanceUnit.MM);
        Pinpoint.setEncoderDirections(Direction(XPodDirection), Direction(YPodDirection));
    }

    /**
     * turns the text from the dashboard into a direction. lenient on purpose: this gets typed live
     * on the laptop, and a typo shouldnt crash the program halfway through a measurement.
     *
     * @param Name "FORWARD" or "REVERSED", any case, spaces around it are fine
     * @return REVERSED for "REVERSED", FORWARD for anything else
     */
    public static GoBildaPinpointDriver.EncoderDirection Direction(String Name) {
        boolean Reversed = Name != null && Name.trim().equalsIgnoreCase("REVERSED");

        return Reversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
}
