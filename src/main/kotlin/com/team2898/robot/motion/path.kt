package com.team2898.robot.motion

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.units.derivedunits.LinearAcceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.LinearVelocity
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.meter
import kotlin.system.measureNanoTime



val testPath = waypoints(
        Pose2d()
)



private val kConstraints = listOf(
        CentripetalAccelerationConstraint(5.5.feet.acceleration)
)
private fun waypoints(vararg waypoints: Pose2d) = listOf(*waypoints)

private fun List<Pose2d>.generateTrajectory(
        name: String,
        reversed: Boolean,
        maxVelocity: LinearVelocity = 10.feet.velocity,
        maxAcceleration: LinearAcceleration = 5.feet.acceleration,
        constraints: List<TimingConstraint<Pose2dWithCurvature>> = kConstraints
): TimedTrajectory<Pose2dWithCurvature> {

    lateinit var trajectory: TimedTrajectory<Pose2dWithCurvature>

    val time = measureNanoTime {
        trajectory = DefaultTrajectoryGenerator.generateTrajectory(
                reversed = reversed,
                wayPoints = this,
                constraints = constraints,
                startVelocity = 0.meter.velocity,
                endVelocity = 0.meter.velocity,
                maxVelocity = maxVelocity,
                maxAcceleration = maxAcceleration
        )
    }
    System.out.printf("Generating %-30s... %3.3f milliseconds.%n", name, time.toDouble() / 1E6)

    return trajectory
}