package org.firstinspires.ftc.teamcode.vision;

public class Motif {
    public boolean[] pattern;

    //function to define the motif for latter
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
