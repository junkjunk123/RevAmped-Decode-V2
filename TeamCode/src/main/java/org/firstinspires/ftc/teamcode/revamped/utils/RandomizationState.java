package org.firstinspires.ftc.teamcode.revamped.utils;

public enum RandomizationState {
    PPG(23),
    PGP(22),
    GPP(21);

    final int ID;

    RandomizationState(int tagID) {
        ID = tagID;
    }

    public int getGreenIndex() {
        switch (this) {
            case PGP -> {
                return 1;
            }
            case PPG -> {
                return 2;
            }
            default -> {
                return 0;
            }
        }
    }

    public int getID() {
        return ID;
    }
}