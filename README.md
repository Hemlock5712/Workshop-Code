# Workshop-Code

Lesson code for the **Gray Matter Workshop** at [frc5712.com](https://frc5712.com).

This project uses the WPILib **2027 alpha** tools: Commands v3, OpModes, Java 25, SystemCore, and
Phoenix 6. Each lesson on the site has a matching numbered branch. To see what one lesson adds,
open the pull request between its branch and the branch before it.

> This branch (`main`) holds the **swerve baseline** — the same code as the
> [v3.0-swerve release](https://github.com/Hemlock5712/Workshop-Code/releases/tag/v3.0-swerve),
> which is the download we recommend for following the workshop.

## Mechanism track (Arm + Flywheel, no drivetrain)

| Branch | Lesson |
| --- | --- |
| `1-Subsystem` | Basic Arm and Flywheel mechanisms — hardware config |
| `2-Commands` | Command factories and controller bindings |
| `3-PID` | Closed-loop PID position/velocity control |
| `4-MotionMagic` | Motion Magic motion profiles |
| `5-GettersAndSetters` | Getters and `isAtTarget` checking |
| `6-Coroutines` | Autonomous routines with coroutines — `fork` / `await` |
| `7-StateBased` | State-based control with the Commands v3 `StateMachine` |

## Swerve track

| Branch | Lesson |
| --- | --- |
| `1-Swerve` | CTRE swerve baseline (this code) |
| `2-Logging` | Logging with DataLogManager |
| `3-Limelight` | Vision with Limelight |
| `4-DynamicFlywheel` | Vision-based shooting |
| `5-DriveToPoint` | Drive to a field point |
| `6-ProfiledToPoint` | Drive to point with profiled PID |
| `7-InlineCommands` | The inline command style — drive to an AprilTag |

## Getting started

1. Install the [WPILib 2027 alpha](https://github.com/wpilibsuite/allwpilib/releases) tools
   (Java comes with them).
2. Download the [swerve baseline](https://github.com/Hemlock5712/Workshop-Code/releases/tag/v3.0-swerve),
   or check out the branch for the lesson you are on.
3. Follow along at [frc5712.com](https://frc5712.com).
