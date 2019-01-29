package com.team2898.robot.commands

import com.team2898.engine.async.AsyncLooper
import com.team2898.engine.controlLoops.modernControl.ramsete
import com.team2898.engine.kinematics.RigidTransform2d
import com.team2898.engine.kinematics.Rotation2d
import com.team2898.engine.kinematics.Translation2d
import com.team2898.engine.kinematics.Twist2d
import com.team2898.engine.math.linear.Matrix
import com.team2898.engine.math.linear.T
import com.team2898.engine.math.linear.get
import com.team2898.engine.math.linear.row
import com.team2898.engine.motion.DriveSignal
import com.team2898.engine.motion.pathfinder.TrajPoint
import com.team2898.robot.subsystem.Drivetrain
import edu.wpi.first.wpilibj.command.Command


class PathFollower(val path: Array<TrajPoint>) : Command() {

    val size = path.size

    var currentIndex = 0

    val X = mutableListOf<Double>()
    val Y = mutableListOf<Double>()

    val auto = AsyncLooper(1/Drivetrain.dt){
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
        val vols = Drivetrain.genU(ref)
        Drivetrain.step(vols)

        currentIndex++
//        println("x: ${target.x}, y: ${target.y}, dtx: ${Drivetrain.state.pose.x}, dty: ${Drivetrain.state.pose.y}, dtv: ${Drivetrain.state.vel.dx}")

        // uncomment those for testing
//        X.add(Drivetrain.state.pose.x)
//        Y.add(Drivetrain.state.pose.y)

        Drivetrain.openLoopPower(DriveSignal(left=vols[0, 0], right=vols[1, 0]))
    }

    override fun isFinished(): Boolean {
        return currentIndex == size
    }

    override fun end() {
        auto.stop()
    }

    override fun start() {
        Drivetrain.encoders { reset() }
        auto.start()
    }
}
