package com.team2898.robot.commands

import com.team2898.engine.controlLoops.modernControl.ramsete
import com.team2898.engine.kinematics.RigidTransform2d
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.kinematics.Translation2d
import com.team2898.engine.kinematics.Twist2d
import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.T
import com.team2898.engine.math.linear.row
import com.team2898.engine.motion.pathfinder.TrajPoint
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.command.Command


class PathFollower(val path: Array<TrajPoint>) : Command() {

    val sb = StringBuilder()
    val size = path.size

    val ramsete_ref = ramsete(
            positionReference = RigidTransform2d(Translation2d(1.0, 0.0), Rotation2d(1.0, 0.0)),
            velocityReference = Twist2d(1.0, 0.0, 0.0),
            pose = RigidTransform2d(Translation2d(0.0, 0.0), Rotation2d(1.0, 0.0)),
            b = 0.5,
            zeta = 0.5,
            wheelbase = Drivetrain.wheelbase.toDouble()
    )


    var currentTime = 0.0
    var currentIndex = 0

    val X = mutableListOf<Double>()
    val Y = mutableListOf<Double>()

    override fun execute() {
        val target = path[currentIndex]
        val ramsete_ref = ramsete(
                positionReference = RigidTransform2d(Translation2d(target.x, target.y), Rotation2d.createFromRadians(target.heading)),
                velocityReference = Twist2d(target.vel, 0.0, target.angVel),
                pose = Drivetrain.state.pose,
                b = 0.6,
                zeta = 0.9,
                wheelbase = 2.0
        )
        val ref = Matrix(Matrix(arrayOf(row(ramsete_ref.first, ramsete_ref.second))).T.data)
        Drivetrain.step(Drivetrain.genU(ref))

        currentTime += Drivetrain.dt

        if (currentTime > target.t) currentIndex++
        //println("${target.vel} ${target.angVel} ${ramsete_ref.first/2 + ramsete_ref.second/2} ${-ramsete_ref.first/Drivetrain.wheelbase + ramsete_ref.second/Drivetrain.wheelbase}")
        //println("ramsete ref: ${ramsete_ref.first}, dt: ${Drivetrain.state.vel.dx - Drivetrain.state.vel.dtheta * Drivetrain.wheelbase / 2}")
        println("x: ${target.x}, y: ${target.y}, dtx: ${Drivetrain.state.pose.x}, dty: ${Drivetrain.state.pose.y}, dtv: ${Drivetrain.state.vel.dx}")
        X.add(Drivetrain.state.pose.x)
        Y.add(Drivetrain.state.pose.y)
    }

    override fun isFinished(): Boolean {
        return currentIndex == size
    }
}
