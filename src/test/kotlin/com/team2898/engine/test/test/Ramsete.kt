package com.team2898.engine.test.test

import com.team2898.engine.kinematics.RigidTransform2d
import com.team2898.engine.kinematics.Twist2d
import com.team2898.engine.math.kEpsilon
import com.team2898.engine.math.sinc
import com.team2898.robot.subsystem.Drivetrain
import kotlin.math.*

/*
def ramsete(pose_desired, v_desired, omega_desired, pose, b, zeta):
    e = pose_desired - pose
    e.rotate(-pose.theta)

    k = 2 * zeta * math.sqrt(omega_desired ** 2 + b * v_desired ** 2)
    v = v_desired * math.cos(e.theta) + k * e.x


    return v, omega
 */

// Twist: x, y, theta
//        v     w

/*

 */
fun ramsete(
        positionReference: RigidTransform2d,
        velocityReference: Twist2d,
        pose: RigidTransform2d,
        b: Double,
        zeta: Double): Pair<Double, Double> {

    // calculating error
    val angleError = positionReference.rotation.rotateBy(pose.rotation.inverse)
    val xError = positionReference.x - pose.x
    val yError = positionReference.y - pose.y

    // getting reference vels
    val vd = velocityReference.dx
    val wd = velocityReference.dtheta

    // gain
    val k1 = 2 * zeta * sqrt(wd * wd + b * vd * vd)
    val k3 = k1

    // getting angle error in radians
    assert(abs(cos(angleError.theta) - angleError.cos) < kEpsilon) { "${cos(angleError.theta)},ree2 ${angleError.cos}" }

    // getting linear and angular vels
    val v = vd * angleError.cos + k1 * (xError * pose.cos + yError * pose.sin)
    val w = wd + b * vd * sinc(angleError.radians) * (yError * pose.cos - yError * pose.sin) + k3 * angleError.radians

    // converting to left and right vel
    val vl = v - w * Drivetrain.wheelbase / 2
    val vr = v + w * Drivetrain.wheelbase / 2

    //println("v: $linearVel, w: $angularVel, vl: $vl, vr: $vr")
    return Pair(vl, vr)
}
