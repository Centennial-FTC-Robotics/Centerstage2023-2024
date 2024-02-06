package org.centennialrobotics.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class CRPoseCalc {

    public static double distance(Pose2d first, Pose2d second) {

        return Math.sqrt(Math.abs(Math.pow(
                first.getX()-second.getX(), 2)+Math.pow(first.getY()-second.getY(), 2)));

    }

}
