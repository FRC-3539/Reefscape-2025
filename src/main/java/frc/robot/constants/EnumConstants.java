package frc.robot.constants;

public class EnumConstants {
    public enum AlgaeMode {
        PROCESSOR, REEFLOW, REEFHIGH, NET, GROUND;
    }

    public enum CoralMode {
        TROUGH, LOW, MID, HIGH;
    }

    public static enum AlignMode {
        A, B, C, D, E, F, G, H, I, J, K, L, HUMANPLAYER1, HUMANPLAYER2, CLIMB1, CLIMB2, CLIMB3, CLOSEST;
    };

    public enum IntakeMode {
        GROUND, HUMAN, REVERSE, HOME, HANDOFF;
    }

    public enum ScoringMode {
        ALGAE, CORAL;
    }
}
