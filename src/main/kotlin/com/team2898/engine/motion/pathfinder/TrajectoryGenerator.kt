//package com.team2898.engine.motion.pathfinder
//
//import com.team2898.engine.kinematics.Rotation2d
//import jaci.pathfinder.Trajectory
//
//
//data class TrajPoint(val t: Double, val x: Double, val y: Double, val vel: Double, val heading: Double, val angVel: Double)
//
//class TrajectoryGenerator {
//    fun getTrajPoints(prof: Trajectory): Array<TrajPoint> {
//        var t = 0.0
//        val dt = prof[0].dt
//        val points: Array<TrajPoint> by lazy {
//            Array<TrajPoint>(prof.length() - 1) { idx ->
//                val line = prof[idx]
//                val nextLine = prof[idx + 1]
//                t += dt
//                val dw = Rotation2d.createFromRadians(nextLine.heading).rotateBy(Rotation2d.createFromRadians(-line.heading)).radians / dt
//                TrajPoint(t = t - dt, x = line.x, y = line.y, heading = line.heading, vel = line.velocity, angVel = dw)
//            }
//        }
//        return points
//    }
//}