package org.firstinspires.ftc.teamcode.vision;

public class Motif {
    public boolean[] pattern;
    public Motif(int ID) {
        switch (ID%20) {
            case 1:
                pattern = new boolean[] {true, false, false};
                break;
            case 2:
                pattern = new boolean[] {false, true, false};
                break;
            case 3:
                pattern = new boolean[] {false, false, true};
                break;
        }
    }
}
