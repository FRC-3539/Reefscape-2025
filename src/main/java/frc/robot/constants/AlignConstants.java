package frc.robot.constants;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlignConstants {
    // Distance of robot center to reef face, in inches
    private final double ALIGN_DISTANCE = 22.0;
    // Robot center offset shift along reef face, in inches
    private final double ALIGN_STRAFE = 6.5;

    public static enum AlignMode {
        A, B, C, D, E, F, G, H, I, J, K, L, HUMANPLAYER1, HUMANPLAYER2, CLIMB1, CLIMB2, CLIMB3, CLOSEST;
    };

    public static Map<AlignMode, Pose2d> coralPoints = new HashMap<>();
    public static Map<AlignMode, Pose2d> algaePoints = new HashMap<>();
    private final double INCHES_TO_METERS = 0.0254;

    public AlignConstants() {
        // Manual points
        coralPoints.put(AlignMode.HUMANPLAYER1, new Pose2d(1.75, 1.75, Rotation2d.fromDegrees(50)));
        coralPoints.put(AlignMode.HUMANPLAYER2, new Pose2d(1.5, 6.75, Rotation2d.fromDegrees(-50)));
        coralPoints.put(AlignMode.CLIMB1, new Pose2d(8.5, 5.25, Rotation2d.fromDegrees(-90)));
        coralPoints.put(AlignMode.CLIMB2, new Pose2d(8.5, 5.25, Rotation2d.fromDegrees(-90)));
        coralPoints.put(AlignMode.CLIMB3, new Pose2d(8.5, 5.25, Rotation2d.fromDegrees(-90)));

        // Calculated points
        coralPoints.put(AlignMode.A, getOffsetPoint(3.6576, 4.0269, 0, -ALIGN_STRAFE));
        coralPoints.put(AlignMode.B, getOffsetPoint(3.6576, 4.0269, 0, +ALIGN_STRAFE));
        coralPoints.put(AlignMode.C, getOffsetPoint(4.0740, 3.3073, 60, -ALIGN_STRAFE));
        coralPoints.put(AlignMode.D, getOffsetPoint(4.0740, 3.3073, 60, +ALIGN_STRAFE));
        coralPoints.put(AlignMode.E, getOffsetPoint(4.9057, 3.3073, 120, -ALIGN_STRAFE));
        coralPoints.put(AlignMode.F, getOffsetPoint(4.9057, 3.3073, 120, +ALIGN_STRAFE));
        coralPoints.put(AlignMode.G, getOffsetPoint(5.3209, 4.0269, 180, -ALIGN_STRAFE));
        coralPoints.put(AlignMode.H, getOffsetPoint(5.3209, 4.0269, 180, +ALIGN_STRAFE));
        coralPoints.put(AlignMode.I, getOffsetPoint(4.9057, 4.7474, -120, -ALIGN_STRAFE));
        coralPoints.put(AlignMode.J, getOffsetPoint(4.9057, 4.7474, -120, +ALIGN_STRAFE));
        coralPoints.put(AlignMode.K, getOffsetPoint(4.0740, 4.7474, -60, -ALIGN_STRAFE));
        coralPoints.put(AlignMode.L, getOffsetPoint(4.0740, 4.7474, -60, +ALIGN_STRAFE));

        // Algae points
        algaePoints.put(AlignMode.A, getOffsetPoint(3.6576, 4.0269, 0, 0));
        algaePoints.put(AlignMode.B, getOffsetPoint(4.0740, 3.3073, 60, 0));
        algaePoints.put(AlignMode.C, getOffsetPoint(4.9057, 3.3073, 120, 0));
        algaePoints.put(AlignMode.D, getOffsetPoint(5.3209, 4.0269, 180, 0));
        algaePoints.put(AlignMode.E, getOffsetPoint(4.9057, 4.7474, -120, 0));
        algaePoints.put(AlignMode.F, getOffsetPoint(4.0740, 4.7474, -60, 0));
    }

    private Pose2d getOffsetPoint(double tagX, double tagY, int robotFaceDirection, double strafe) {
        double tagToPointDistance = Math.sqrt(Math.pow(ALIGN_DISTANCE, 2) + Math.pow(strafe, 2)) * INCHES_TO_METERS;
        double pointOffsetAngle = Math.asin(strafe * INCHES_TO_METERS / tagToPointDistance);
        double reefFaceOutwardDirection = Math.toRadians(robotFaceDirection + 180) + pointOffsetAngle;
        return new Pose2d(
            tagX + Math.cos(reefFaceOutwardDirection) * tagToPointDistance,
            tagY + Math.sin(reefFaceOutwardDirection) * tagToPointDistance,
            Rotation2d.fromDegrees(robotFaceDirection));
    }
}

// points.put(AlignMode.A, new Pose2d(3.10, 4.19, Rotation2d.fromDegrees(0)));
// points.put(AlignMode.B, new Pose2d(3.10, 3.86, Rotation2d.fromDegrees(0)));
// points.put(AlignMode.C, new Pose2d(3.65, 2.90, Rotation2d.fromDegrees(60)));
// points.put(AlignMode.D, new Pose2d(3.93, 2.74, Rotation2d.fromDegrees(60)));
// points.put(AlignMode.E, new Pose2d(5.04, 2.74, Rotation2d.fromDegrees(120)));
// points.put(AlignMode.F, new Pose2d(5.33, 2.90, Rotation2d.fromDegrees(120)));
// points.put(AlignMode.G, new Pose2d(5.88, 3.86, Rotation2d.fromDegrees(180)));
// points.put(AlignMode.H, new Pose2d(5.88, 4.19, Rotation2d.fromDegrees(180)));
// points.put(AlignMode.I, new Pose2d(5.33, 5.15, Rotation2d.fromDegrees(-120)));
// points.put(AlignMode.J, new Pose2d(5.04, 5.31, Rotation2d.fromDegrees(-120)));
// points.put(AlignMode.K, new Pose2d(3.93, 5.31, Rotation2d.fromDegrees(-60)));
// points.put(AlignMode.L, new Pose2d(3.65, 5.15, Rotation2d.fromDegrees(-60)));
// points.put(AlignMode.HUMANPLAYER1, new Pose2d(1.75, 1.75, Rotation2d.fromDegrees(50)));
// points.put(AlignMode.HUMANPLAYER2, new Pose2d(1.5, 6.75, Rotation2d.fromDegrees(-50)));
// points.put(AlignMode.CLIMB1, new Pose2d(8.5, 5.25, Rotation2d.fromDegrees(-90)));
// points.put(AlignMode.CLIMB2, new Pose2d(8.5, 5.25, Rotation2d.fromDegrees(-90)));
// points.put(AlignMode.CLIMB3, new Pose2d(8.5, 5.25, Rotation2d.fromDegrees(-90)));