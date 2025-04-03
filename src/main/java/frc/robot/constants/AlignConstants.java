package frc.robot.constants;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.EnumConstants.*;


public class AlignConstants {
    // Distance of robot center to reef face, in inches
    private final double ALIGN_DISTANCE = 22.0;
    // Robot center offset shift along reef face, in inches
    private final double ALIGN_STRAFE = 6.5;

    private final double ALGAE_STRAIGHT_DISTANCE = 12;

    

    public static Map<AlignPoint, Pose2d> coralPointsLeft = new HashMap<>();
    public static Map<AlignPoint, Pose2d> coralPointsRight = new HashMap<>();
    public static Map<AlignPoint, Pose2d> algaePoints = new HashMap<>();
    public static Map<AlignPoint, Pose2d> straightPoints = new HashMap<>();
    public static Map<AlignPoint, Pose2d> climbPoints = new HashMap<>();
    public static Map<AlignPoint, Pose2d> humanPlayerPoints = new HashMap<>();
    private final double INCHES_TO_METERS = 0.0254;

    public AlignConstants() {
        // Manual points
        humanPlayerPoints.put(AlignPoint.HUMANPLAYER1, new Pose2d(1.73, 7.17, Rotation2d.fromDegrees(-52.5)));
        // Bumper 9.5" from human player wall
        // Robot Frame 12.75" from human player wall
        humanPlayerPoints.put(AlignPoint.HUMANPLAYER2, new Pose2d(1.73, 0.88, Rotation2d.fromDegrees(52.5)));
        climbPoints.put(AlignPoint.CLIMB1, new Pose2d(7.260,7.163, Rotation2d.fromDegrees(-90)));
        climbPoints.put(AlignPoint.CLIMB2, new Pose2d(7.260, 6.068, Rotation2d.fromDegrees(-90)));
        climbPoints.put(AlignPoint.CLIMB3, new Pose2d(7.260, 4.973, Rotation2d.fromDegrees(-90)));

        // Calculated points
        coralPointsLeft.put(AlignPoint.A, getOffsetPoint(3.6576, 4.0269, 0, -ALIGN_STRAFE, ALIGN_DISTANCE));
        coralPointsRight.put(AlignPoint.B, getOffsetPoint(3.6576, 4.0269, 0, +ALIGN_STRAFE, ALIGN_DISTANCE));
        coralPointsLeft.put(AlignPoint.C, getOffsetPoint(4.0740, 3.3073, 60, -ALIGN_STRAFE, ALIGN_DISTANCE));
        coralPointsRight.put(AlignPoint.D, getOffsetPoint(4.0740, 3.3073, 60, +ALIGN_STRAFE, ALIGN_DISTANCE));
        coralPointsLeft.put(AlignPoint.E, getOffsetPoint(4.9057, 3.3073, 120, -ALIGN_STRAFE, ALIGN_DISTANCE));
        coralPointsRight.put(AlignPoint.F, getOffsetPoint(4.9057, 3.3073, 120, +ALIGN_STRAFE, ALIGN_DISTANCE));
        coralPointsLeft.put(AlignPoint.G, getOffsetPoint(5.3209, 4.0269, 180, -ALIGN_STRAFE, ALIGN_DISTANCE));
        coralPointsRight.put(AlignPoint.H, getOffsetPoint(5.3209, 4.0269, 180, +ALIGN_STRAFE, ALIGN_DISTANCE));
        coralPointsLeft.put(AlignPoint.I, getOffsetPoint(4.9057, 4.7474, -120, -ALIGN_STRAFE, ALIGN_DISTANCE));
        coralPointsRight.put(AlignPoint.J, getOffsetPoint(4.9057, 4.7474, -120, +ALIGN_STRAFE, ALIGN_DISTANCE));
        coralPointsLeft.put(AlignPoint.K, getOffsetPoint(4.0740, 4.7474, -60, -ALIGN_STRAFE, ALIGN_DISTANCE));
        coralPointsRight.put(AlignPoint.L, getOffsetPoint(4.0740, 4.7474, -60, +ALIGN_STRAFE, ALIGN_DISTANCE));

        // Algae points
        algaePoints.put(AlignPoint.A, getOffsetPoint(3.6576, 4.0269, 0, 0, ALIGN_DISTANCE));
        algaePoints.put(AlignPoint.B, getOffsetPoint(4.0740, 3.3073, 60, 0, ALIGN_DISTANCE));
        algaePoints.put(AlignPoint.C, getOffsetPoint(4.9057, 3.3073, 120, 0, ALIGN_DISTANCE));
        algaePoints.put(AlignPoint.D, getOffsetPoint(5.3209, 4.0269, 180, 0, ALIGN_DISTANCE));
        algaePoints.put(AlignPoint.E, getOffsetPoint(4.9057, 4.7474, -120, 0, ALIGN_DISTANCE));
        algaePoints.put(AlignPoint.F, getOffsetPoint(4.0740, 4.7474, -60, 0, ALIGN_DISTANCE));

        straightPoints.put(AlignPoint.A, getOffsetPoint(3.6576, 4.0269, 0, 0, ALGAE_STRAIGHT_DISTANCE));
        straightPoints.put(AlignPoint.B, getOffsetPoint(4.0740, 3.3073, 60, 0, ALGAE_STRAIGHT_DISTANCE));
        straightPoints.put(AlignPoint.C, getOffsetPoint(4.9057, 3.3073, 120, 0, ALGAE_STRAIGHT_DISTANCE));
        straightPoints.put(AlignPoint.D, getOffsetPoint(5.3209, 4.0269, 180, 0, ALGAE_STRAIGHT_DISTANCE));
        straightPoints.put(AlignPoint.E, getOffsetPoint(4.9057, 4.7474, -120, 0, ALGAE_STRAIGHT_DISTANCE));
        straightPoints.put(AlignPoint.F, getOffsetPoint(4.0740, 4.7474, -60, 0, ALGAE_STRAIGHT_DISTANCE));
    }

    private Pose2d getOffsetPoint(double tagX, double tagY, int robotFaceDirection, double strafe, double distance) {
        double tagToPointDistance = Math.sqrt(Math.pow(distance, 2) + Math.pow(strafe, 2)) * INCHES_TO_METERS;
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