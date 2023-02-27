package frc.robot.util;

public class TurnTo {
    private static double[] alignBounds = { 15, 60, 120, 165, 195, 240, 300, 345 };
    private static double[] alignAngles90 = { 0, 90, 180, 270 };
    private static double[] alignAngles30 = { 30, 150, 210, 325 };

    public static double align(double currAngle, boolean rightAngles) {
        if (rightAngles) {
            if (currAngle >= 315 || currAngle <= 45) {
                return alignAngles90[0];
            } else if (between(currAngle, 45, 135)) {
                return alignAngles90[1];
            } else if (between(currAngle, 135, 225)) {
                return alignAngles90[2];
            } else if (between(currAngle, 225, 315)) {
                return alignAngles90[3];
            } else {
                return 0;
            }
        } else {
            if (between(currAngle, 0, 90)) {
                return alignAngles30[0];
            } else if (between(currAngle, 90, 180)) {
                return alignAngles30[1];
            } else if (between(currAngle, 180, 270)) {
                return alignAngles30[2];
            } else if (between(currAngle, 270, 359)) {
                return alignAngles30[3];
            } else {
                return 0;
            }
        }
        /*
         * if(currAngle >= alignBounds[7] || currAngle <= alignBounds[0]){
         * return alignAngles[0];
         * }else if(between(currAngle, alignBounds[0], alignBounds[1])){
         * return alignAngles[1];
         * }else if(between(currAngle, alignBounds[1], alignBounds[2])){
         * return alignAngles[2];
         * }else if(between(currAngle, alignBounds[2], alignBounds[3])){
         * return alignAngles[3];
         * }else if(between(currAngle, alignBounds[3], alignBounds[4])){
         * return alignAngles[4];
         * }else if(between(currAngle, alignBounds[4], alignBounds[5])){
         * return alignAngles[5];
         * }else if(between(currAngle, alignBounds[5], alignBounds[6])){
         * return alignAngles[6];
         * }else if(between(currAngle, alignBounds[6], alignBounds[7])){
         * return alignAngles[7];
         * }else{
         * return 0;
         * }
         */
    }

    private static boolean between(double key, double min, double max) {
        if (key >= min && key <= max) {
            return true;
        } else {
            return false;
        }
    }
}
