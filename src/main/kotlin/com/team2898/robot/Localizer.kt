package com.team2898.robot

import com.team2898.engine.async.NotifierLooper
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.kinematics.Translation2d
import com.team2898.engine.math.linear.get
import com.team2898.engine.subsystems.Navx
import com.team2898.robot.subsystem.Drivetrain
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.feet

object Localizer {
    val dt = 0.01

    val currentPose2d: Pose2d = Pose2d(0.0.feet, 0.0.feet, 0.0.degree)


    init {
        NotifierLooper(1/dt) {
            val l = Drivetrain.x[0, 0]
            val r = Drivetrain.x[1, 0]
            val v = (l + r) / 2
            val w = (r - l) / Drivetrain.wheelbase
            Translation2d(0.0, v).rotateByOrigin(Rotation2d.createFromDegrees(Navx.yaw))
        }
    }


}