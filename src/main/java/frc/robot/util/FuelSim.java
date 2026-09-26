// https://github.com/hammerheads5000/FuelSim
package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FuelSim {
    protected static final double PERIOD = 0.02; // sec
    protected static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81); // m/s^2
    // Room temperature dry air density: https://en.wikipedia.org/wiki/Density_of_air#Dry_air
    protected static final double AIR_DENSITY = 1.2041; // kg/m^3
    protected static final double FIELD_COR = Math.sqrt(22 / 51.5); // coefficient of restitution with the field
    protected static final double FUEL_COR = 0.5; // coefficient of restitution with another fuel
    protected static final double NET_COR = 0.2; // coefficient of restitution with the net
    protected static final double ROBOT_COR = 0.1; // coefficient of restitution with a robot
    protected static final double FUEL_RADIUS = 0.075;
    protected static final double SLEEP_SPEED = 0.03;
    protected static final double WAKE_IMPACT_SPEED = 0.35;
    protected static final int SLEEP_TICKS = 10;
    protected static final double FIELD_LENGTH = 16.51;
    protected static final double FIELD_WIDTH = 8.04;
    protected static final double TRENCH_WIDTH = 1.265;
    protected static final double TRENCH_BLOCK_WIDTH = 0.305;
    protected static final double TRENCH_HEIGHT = 0.565;
    protected static final double TRENCH_BAR_HEIGHT = 0.102;
    protected static final double TRENCH_BAR_WIDTH = 0.152;
    protected static final double FRICTION = 0.1; // proportion of horizontal vel to lose per sec while on ground
    protected static final double FUEL_MASS = 0.448 * 0.45392; // kgs
    protected static final double FUEL_CROSS_AREA = Math.PI * FUEL_RADIUS * FUEL_RADIUS;
    // Drag coefficient of smooth sphere: https://en.wikipedia.org/wiki/Drag_coefficient#/media/File:14ilf1l.svg
    protected static final double DRAG_COF = 0.47; // dimensionless
    protected static final double DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA;

    protected static final Translation3d[] FIELD_XZ_LINE_STARTS = {
        new Translation3d(0, 0, 0),
        new Translation3d(3.96, 1.57, 0),
        new Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
        new Translation3d(4.61, 1.57, 0.165),
        new Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
        new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
        new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
        new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
        new Translation3d(3.96, TRENCH_WIDTH, TRENCH_HEIGHT),
        new Translation3d(3.96, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
        new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, TRENCH_HEIGHT),
        new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
        new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(
                FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    };

    protected static final Translation3d[] FIELD_XZ_LINE_ENDS = {
        new Translation3d(FIELD_LENGTH, FIELD_WIDTH, 0),
        new Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
        new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
        new Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0),
        new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0),
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0),
        new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
        new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
        new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
        new Translation3d(
                4.61 + TRENCH_BAR_WIDTH / 2, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(
                FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
                TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
        new Translation3d(FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    };

    protected static class Fuel {
        protected Translation3d pos;
        protected Translation3d vel;
        protected double spin;
        protected boolean asleep = false;
        protected int restTicks = 0;
        protected double hubReturn = 0.0;
        protected Translation3d hubEntry = new Translation3d();
        protected Translation3d hubExit = new Translation3d();
        protected Translation3d hubExitVel = new Translation3d();

        protected Fuel(Translation3d pos, Translation3d vel, double spin) {
            this.pos = pos;
            this.vel = vel;
            this.spin = spin;
        }

        protected Fuel(Translation3d pos, Translation3d vel) {
            this(pos, vel, 0.0);
        }

        protected Fuel(Translation3d pos) {
            this(pos, new Translation3d());
        }

        protected void update(boolean simulateAirResistance, int subticks,
                double dragK, double liftK, double groundFriction) {
            if (hubReturn > 0.0) {
                // Scored fuel rides down the inside of the hub to the return chute. Dropping it
                // straight onto the chute moved it a metre and a half in one frame, which reads
                // as a fuel teleporting across the field. Interpolating the trip rather than
                // integrating it keeps the arrival exact even if a collision pass nudged the fuel
                // on the tick it scored.
                hubReturn -= PERIOD / subticks;
                if (hubReturn <= 0.0) {
                    pos = hubExit;
                    vel = hubExitVel;
                } else {
                    pos = hubEntry.interpolate(hubExit, 1.0 - hubReturn / Hub.RETURN_TIME);
                }
                return;
            }
            pos = pos.plus(vel.times(PERIOD / subticks));
            if (pos.getZ() > FUEL_RADIUS) {
                Translation3d Fg = GRAVITY.times(FUEL_MASS);
                Translation3d Fd = new Translation3d();

                if (simulateAirResistance) {
                    double speed = vel.getNorm();
                    if (speed > 1e-6) {
                        Fd = vel.times(-DRAG_FORCE_FACTOR * speed);
                    }
                }

                Translation3d accel = Fg.plus(Fd).div(FUEL_MASS);
                if (dragK > 0.0) {
                    accel = GRAVITY.plus(quadraticDragWithMagnus(vel, spin, dragK, liftK));
                }
                vel = vel.plus(accel.times(PERIOD / subticks));
            }
            if (Math.abs(vel.getZ()) < 0.05 && pos.getZ() <= FUEL_RADIUS + 0.03) {
                vel = new Translation3d(vel.getX(), vel.getY(), 0);
                vel = vel.times(1 - groundFriction * PERIOD / subticks);
                // pos = new Translation3d(pos.getX(), pos.getY(), FUEL_RADIUS);
            }
            handleFieldCollisions(subticks);

            // A fuel that has settled is put to sleep: it stops being integrated, and its
            // position stops changing at all, so the resting pile no longer has to be
            // republished every loop. Anything that touches it wakes it up again.
            if (pos.getZ() <= FUEL_RADIUS + 0.01 && vel.getNorm() < SLEEP_SPEED) {
                if (++restTicks >= SLEEP_TICKS) {
                    pos = new Translation3d(pos.getX(), pos.getY(), FUEL_RADIUS);
                    vel = new Translation3d();
                    spin = 0.0;
                    asleep = true;
                }
            } else {
                restTicks = 0;
            }
        }

        protected void handleXZLineCollision(Translation3d lineStart, Translation3d lineEnd) {
            if (pos.getY() < lineStart.getY() || pos.getY() > lineEnd.getY()) return; // not within y range
            // Convert into 2D
            Translation2d start2d = new Translation2d(lineStart.getX(), lineStart.getZ());
            Translation2d end2d = new Translation2d(lineEnd.getX(), lineEnd.getZ());
            Translation2d pos2d = new Translation2d(pos.getX(), pos.getZ());
            Translation2d lineVec = end2d.minus(start2d);

            // Get closest point on line
            Translation2d projected =
                    start2d.plus(lineVec.times(pos2d.minus(start2d).dot(lineVec) / lineVec.getSquaredNorm()));

            if (projected.getDistance(start2d) + projected.getDistance(end2d) > lineVec.getNorm())
                return; // projected point not on line
            double dist = pos2d.getDistance(projected);
            if (dist > FUEL_RADIUS) return; // not intersecting line
            // Back into 3D
            Translation3d normal = new Translation3d(-lineVec.getY(), 0, lineVec.getX()).div(lineVec.getNorm());

            // Apply collision response
            pos = pos.plus(normal.times(FUEL_RADIUS - dist));
            if (vel.dot(normal) > 0) return; // already moving away from line
            vel = vel.minus(normal.times((1 + FIELD_COR) * vel.dot(normal)));
        }

        protected void handleFieldCollisions(int subticks) {
            // floor and bumps
            for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
                handleXZLineCollision(FIELD_XZ_LINE_STARTS[i], FIELD_XZ_LINE_ENDS[i]);
            }

            // edges
            if (pos.getX() < FUEL_RADIUS && vel.getX() < 0) {
                pos = pos.plus(new Translation3d(FUEL_RADIUS - pos.getX(), 0, 0));
                vel = vel.plus(new Translation3d(-(1 + FIELD_COR) * vel.getX(), 0, 0));
            } else if (pos.getX() > FIELD_LENGTH - FUEL_RADIUS && vel.getX() > 0) {
                pos = pos.plus(new Translation3d(FIELD_LENGTH - FUEL_RADIUS - pos.getX(), 0, 0));
                vel = vel.plus(new Translation3d(-(1 + FIELD_COR) * vel.getX(), 0, 0));
            }

            if (pos.getY() < FUEL_RADIUS && vel.getY() < 0) {
                pos = pos.plus(new Translation3d(0, FUEL_RADIUS - pos.getY(), 0));
                vel = vel.plus(new Translation3d(0, -(1 + FIELD_COR) * vel.getY(), 0));
            } else if (pos.getY() > FIELD_WIDTH - FUEL_RADIUS && vel.getY() > 0) {
                pos = pos.plus(new Translation3d(0, FIELD_WIDTH - FUEL_RADIUS - pos.getY(), 0));
                vel = vel.plus(new Translation3d(0, -(1 + FIELD_COR) * vel.getY(), 0));
            }

            // hubs
            handleHubCollisions(Hub.BLUE_HUB, subticks);
            handleHubCollisions(Hub.RED_HUB, subticks);

            handleTrenchCollisions();
        }

        protected void wake() {
            asleep = false;
            restTicks = 0;
        }

        protected void handleHubCollisions(Hub hub, int subticks) {
            if (hubReturn > 0.0) return; // already on its way down to the chute
            hub.handleHubInteraction(this, subticks);
            if (hubReturn > 0.0) return; // just scored, the hub walls no longer apply
            hub.fuelCollideSide(this);

            double netCollision = hub.fuelHitNet(this);
            if (netCollision != 0) {
                pos = pos.plus(new Translation3d(netCollision, 0, 0));
                vel = new Translation3d(-vel.getX() * NET_COR, vel.getY() * NET_COR, vel.getZ());
            }
        }

        protected void handleTrenchCollisions() {
            fuelCollideRectangle(
                    this,
                    new Translation3d(3.96, TRENCH_WIDTH, 0),
                    new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(3.96, FIELD_WIDTH - 1.57, 0),
                    new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, 0),
                    new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, 0),
                    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
                    new Translation3d(
                            4.61 + TRENCH_BAR_WIDTH / 2,
                            TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                            TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
                    new Translation3d(4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
                    new Translation3d(
                            FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
                            TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                            TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
            fuelCollideRectangle(
                    this,
                    new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
                    new Translation3d(
                            FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
                            FIELD_WIDTH,
                            TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
        }

        protected void addImpulse(Translation3d impulse) {
            vel = vel.plus(impulse);
        }
    }

    protected static Translation3d quadraticDragWithMagnus(
            Translation3d v, double spin, double dragK, double liftK) {
        double speed = v.getNorm();
        if (speed < 1e-6) {
            return new Translation3d();
        }
        Translation3d drag = v.times(-dragK * speed);
        double spinRatio = Math.abs(spin) * FUEL_RADIUS / speed;
        double liftCoefficient = spinRatio / (2.0 + spinRatio);
        double vh = Math.hypot(v.getX(), v.getY());
        if (liftCoefficient == 0.0 || vh < 1e-6) {
            return drag;
        }
        double k = liftK * liftCoefficient * speed;
        return drag.plus(new Translation3d(
                -k * v.getZ() * v.getX() / vh,
                -k * v.getZ() * v.getY() / vh,
                k * vh));
    }

    /** Resolve against a settled fuel treated as immovable, leaving it asleep. */
    protected static void bounceOffSleeper(Fuel mover, Fuel sleeper) {
        Translation3d normal = mover.pos.minus(sleeper.pos);
        double distance = normal.getNorm();
        if (distance == 0) {
            normal = new Translation3d(0, 0, 1);
            distance = 1;
        }
        normal = normal.div(distance);
        mover.pos = mover.pos.plus(normal.times(FUEL_RADIUS * 2 - distance));
        double into = mover.vel.dot(normal);
        if (into < 0) {
            mover.addImpulse(normal.times(-(1 + FUEL_COR) * into));
        }
    }

    protected static void handleFuelCollision(Fuel a, Fuel b) {
        Translation3d normal = a.pos.minus(b.pos);
        double distance = normal.getNorm();
        if (distance == 0) {
            normal = new Translation3d(1, 0, 0);
            distance = 1;
        }
        normal = normal.div(distance);
        double impulse = 0.5 * (1 + FUEL_COR) * (b.vel.minus(a.vel).dot(normal));
        double intersection = FUEL_RADIUS * 2 - distance;
        a.pos = a.pos.plus(normal.times(intersection / 2));
        b.pos = b.pos.minus(normal.times(intersection / 2));
        a.addImpulse(normal.times(impulse));
        b.addImpulse(normal.times(-impulse));
    }

    protected static final double CELL_SIZE = 0.25;
    protected static final int GRID_COLS = (int) Math.ceil(FIELD_LENGTH / CELL_SIZE);
    protected static final int GRID_ROWS = (int) Math.ceil(FIELD_WIDTH / CELL_SIZE);

    @SuppressWarnings("unchecked")
    protected final ArrayList<Fuel>[][] grid = new ArrayList[GRID_COLS][GRID_ROWS];

    private final ArrayList<ArrayList<Fuel>> activeCells = new ArrayList<>();

    protected void handleFuelCollisions(ArrayList<Fuel> fuels) {
        // Clear grid
        for (ArrayList<Fuel> cell : activeCells) {
            cell.clear();
        }
        activeCells.clear();

        // Populate grid
        for (Fuel fuel : fuels) {
            int col = (int) (fuel.pos.getX() / CELL_SIZE);
            int row = (int) (fuel.pos.getY() / CELL_SIZE);

            if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
                grid[col][row].add(fuel);
                if (grid[col][row].size() == 1) {
                    activeCells.add(grid[col][row]);
                }
            }
        }

        // Check collisions
        for (Fuel fuel : fuels) {
            int col = (int) (fuel.pos.getX() / CELL_SIZE);
            int row = (int) (fuel.pos.getY() / CELL_SIZE);

            // Check 3x3 neighbor cells
            for (int i = col - 1; i <= col + 1; i++) {
                for (int j = row - 1; j <= row + 1; j++) {
                    if (i >= 0 && i < GRID_COLS && j >= 0 && j < GRID_ROWS) {
                        for (Fuel other : grid[i][j]) {
                            if (fuel != other && fuel.pos.getDistance(other.pos) < FUEL_RADIUS * 2) {
                                if (fuel.hashCode() < other.hashCode()) {
                                    // Two settled fuel resting against each other are a pile,
                                    // not a collision - waking them here churned the whole pile
                                    // awake and back to sleep every loop.
                                    if (fuel.asleep && other.asleep) continue;
                                    if (fuel.asleep || other.asleep) {
                                        Fuel sleeper = fuel.asleep ? fuel : other;
                                        Fuel mover = fuel.asleep ? other : fuel;
                                        if (mover.vel.getNorm() < WAKE_IMPACT_SPEED) {
                                            // Nudging a settled heap does not disturb it: the
                                            // sleeper acts like ground, so the arriving fuel
                                            // rolls to a stop on it instead of starting a
                                            // cascade that keeps the whole pile awake.
                                            bounceOffSleeper(mover, sleeper);
                                            continue;
                                        }
                                        sleeper.wake();
                                        restingDirty = true;
                                    }
                                    handleFuelCollision(fuel, other);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    protected ArrayList<Fuel> fuels = new ArrayList<>();
    protected boolean restingDirty = true;
    protected int maxFuel = 500;
    protected int restingPublishes = 0;
    protected boolean running = false;
    protected boolean simulateAirResistance = false;
    protected Supplier<Pose2d> robotPoseSupplier = null;
    protected Supplier<ChassisSpeeds> robotFieldSpeedsSupplier = null;
    protected double robotWidth; // size along the robot's y axis
    protected double robotLength; // size along the robot's x axis
    protected double bumperHeight;
    protected ArrayList<SimIntake> intakes = new ArrayList<>();
    protected int subticks = 5;
    protected double dragK = 0.0;
    protected double liftK = 0.0;
    protected double groundFriction = FRICTION;
    protected double loggingFreqHz = 10;
    protected Timer loggingTimer = new Timer();

    /**
     * Creates a new instance of FuelSim
     * @param tableKey NetworkTable to log fuel positions to as an array of {@link Translation3d} structs.
     */
    public FuelSim(String tableKey) {
        // Initialize grid
        for (int i = 0; i < GRID_COLS; i++) {
            for (int j = 0; j < GRID_ROWS; j++) {
                grid[i][j] = new ArrayList<Fuel>();
            }
        }

        this.fuelLogKey = tableKey + "/Fuels";
    }

    /**
     * Creates a new instance of FuelSim with log path "FuelSim"
     */
    public FuelSim() {
        this("FuelSim");
    }

    /**
     * Clears the field of fuel
     */
    public void clearFuel() {
        restingDirty = true;
        fuels.clear();
    }

    /**
     * Spawns fuel in the neutral zone and depots
     */
    public void spawnStartingFuel() {
        // Center fuel
        Translation3d center = new Translation3d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, FUEL_RADIUS);
        for (int i = 0; i < 15; i++) {
            for (int j = 0; j < 6; j++) {
                fuels.add(new Fuel(center.plus(new Translation3d(0.076 + 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0))));
                fuels.add(new Fuel(center.plus(new Translation3d(-0.076 - 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0))));
                fuels.add(new Fuel(center.plus(new Translation3d(0.076 + 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0))));
                fuels.add(new Fuel(center.plus(new Translation3d(-0.076 - 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0))));
            }
        }

        // Depots
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                fuels.add(new Fuel(new Translation3d(0.076 + 0.152 * j, 5.95 + 0.076 + 0.152 * i, FUEL_RADIUS)));
                fuels.add(new Fuel(new Translation3d(0.076 + 0.152 * j, 5.95 - 0.076 - 0.152 * i, FUEL_RADIUS)));
                fuels.add(new Fuel(
                        new Translation3d(FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 + 0.076 + 0.152 * i, FUEL_RADIUS)));
                fuels.add(new Fuel(
                        new Translation3d(FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 - 0.076 - 0.152 * i, FUEL_RADIUS)));
            }
        }

        // DEBUG: Log XZ lines
        // Translation3d[][] lines = new Translation3d[FIELD_XZ_LINE_STARTS.length][2];
        // for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
        //     lines[i][0] = FIELD_XZ_LINE_STARTS[i];
        //     lines[i][1] = FIELD_XZ_LINE_ENDS[i];
        // }

        // Logger.recordOutput("Fuel Simulation/Lines (debug)", lines);
    }

    protected final String fuelLogKey;

    /**
     * Records the array of `Translation3d`'s to the AdvantageKit log at tableKey + "/Fuels".
     * AdvantageKit's NT4Publisher republishes it live, so it is visible in AdvantageScope
     * both over NetworkTables and when replaying the .wpilog.
     */
    public void logFuels() {
        // Fuel is split by whether it is awake, not by whether it is airborne. Moving fuel goes
        // out every loop so shots stay smooth; the resting pile is republished only when it
        // actually changes, because sending 400 unchanged positions 50 times a second is what
        // makes a long session crawl. A fuel lands and settles in the same place it stopped, so
        // handing it between the two keys shows no jump.
        int awake = 0;
        for (Fuel fuel : fuels) {
            if (!fuel.asleep) awake++;
        }
        Translation3d[] moving = new Translation3d[awake];
        int m = 0;
        for (Fuel fuel : fuels) {
            if (!fuel.asleep) moving[m++] = fuel.pos;
        }
        Logger.recordOutput(fuelLogKey + "Moving", moving);

        if (restingDirty) {
            Translation3d[] resting = new Translation3d[fuels.size() - awake];
            int r = 0;
            for (Fuel fuel : fuels) {
                if (fuel.asleep) resting[r++] = fuel.pos;
            }
            Logger.recordOutput(fuelLogKey, resting);
            restingDirty = false;
            restingPublishes++;
        }
        Logger.recordOutput(fuelLogKey + "Count", fuels.size());
    }

    /** @return how many times the resting pile has been republished since boot */
    public int getRestingPublishes() {
        return restingPublishes;
    }

    /** @return how many fuel are awake and therefore being simulated this loop */
    public int getAwakeCount() {
        int awake = 0;
        for (Fuel fuel : fuels) {
            if (!fuel.asleep) awake++;
        }
        return awake;
    }

    public void logFuelsInFlight() {
        Translation3d[] airborne = fuels.stream()
                .filter(FuelSim::isInFlight)
                .map((fuel) -> fuel.pos)
                .toArray(Translation3d[]::new);
        Logger.recordOutput(fuelLogKey + "InFlight", airborne);
        Logger.recordOutput(fuelLogKey + "InFlightCount", airborne.length);
    }

    public int getFuelInFlightCount() {
        return (int) fuels.stream().filter(FuelSim::isInFlight).count();
    }

    protected static boolean isInFlight(Fuel fuel) {
        return fuel.pos.getZ() > FUEL_RADIUS + 0.03 && fuel.vel.getNorm() > 0.05;
    }

    /**
     * @return the number of fuel currently on the field
     */
    public int getFuelCount() {
        return fuels.size();
    }

    /** @return every fuel's position, in the same order and content as the logged array */
    public Translation3d[] getFuelPositions() {
        return fuels.stream().map(fuel -> fuel.pos).toArray(Translation3d[]::new);
    }

    /**
     * LOCAL ADDITION. Sets how fast a fuel on the ground sheds horizontal speed, as a fraction
     * per second. Upstream's 0.1 loses under 10% per second, so a ball skates across the whole
     * field before it settles. Higher is grippier.
     *
     * @param perSecond fraction of horizontal velocity lost per second while rolling
     */
    public void setGroundFriction(double perSecond) {
        this.groundFriction = perSecond;
    }

    /**
     * LOCAL ADDITION. Drops {@code count} fuel in a single grid pile centred on the given point,
     * at rest on the floor. Upstream's spawnStartingFuel also stocks both depots; this is for
     * when you only want one pile to drive into.
     *
     * @param centreX field X of the pile centre
     * @param centreY field Y of the pile centre
     * @param count how many fuel to place
     * @param spacing centre-to-centre spacing in metres
     */
    public void spawnPile(double centreX, double centreY, int count, double spacing) {
        int cols = (int) Math.ceil(Math.sqrt(count));
        for (int i = 0; i < count; i++) {
            int col = i % cols;
            int row = i / cols;
            fuels.add(new Fuel(new Translation3d(
                    centreX + (col - (cols - 1) / 2.0) * spacing,
                    centreY + (row - (Math.ceil((double) count / cols) - 1) / 2.0) * spacing,
                    FUEL_RADIUS)));
        }
    }

    /**
     * LOCAL ADDITION. Thins the field down to at most {@code count} fuel, keeping an evenly
     * spaced subset so the depots and the neutral pile stay proportionally represented.
     * A full field is 408 fuel, and every one of them is a sphere the renderer has to draw.
     *
     * @param count maximum fuel to leave on the field
     */
    public void thinTo(int count) {
        if (count >= fuels.size() || count < 0) {
            return;
        }
        ArrayList<Fuel> kept = new ArrayList<>(count);
        double stride = (double) fuels.size() / count;
        for (int i = 0; i < count; i++) {
            kept.add(fuels.get((int) (i * stride)));
        }
        fuels = kept;
    }

    /**
     * Start the simulation. `updateSim` must still be called every loop
     */
    public void start() {
        running = true;
        loggingTimer.restart();
    }

    /**
     * Pause the simulation.
     */
    public void stop() {
        running = false;
        loggingTimer.stop();
    }

    /** Enables accounting for drag force in physics step **/
    public void enableAirResistance() {
        simulateAirResistance = true;
    }

    public void useQuadraticDragWithMagnus(double dragK, double liftK) {
        this.dragK = dragK;
        this.liftK = liftK;
        this.simulateAirResistance = false;
    }

    /**
     * Sets the number of physics iterations per loop (0.02s)
     * @param subticks physics iteration per loop (default: 5)
     */
    public void setSubticks(int subticks) {
        this.subticks = subticks;
    }

    /**
     * Sets the frequency to publish fuel translations to NetworkTables
     * Used to improve performance in AdvantageScope
     * @param loggingFreqHz update frequency in hertz
     */
    public void setLoggingFrequency(double loggingFreqHz) {
        this.loggingFreqHz = loggingFreqHz;
    }

    /**
     * Registers a robot with the fuel simulator
     * @param width from left to right (y-axis)
     * @param length from front to back (x-axis)
     * @param bumperHeight
     * @param poseSupplier
     * @param fieldSpeedsSupplier field-relative `ChassisSpeeds` supplier
     */
    public void registerRobot(
            double width,
            double length,
            double bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.robotPoseSupplier = poseSupplier;
        this.robotFieldSpeedsSupplier = fieldSpeedsSupplier;
        this.robotWidth = width;
        this.robotLength = length;
        this.bumperHeight = bumperHeight;
    }

    /**
     * Registers a robot with the fuel simulator
     * @param width from left to right (y-axis)
     * @param length from front to back (x-axis)
     * @param bumperHeight from the ground
     * @param poseSupplier
     * @param fieldSpeedsSupplier field-relative `ChassisSpeeds` supplier
     */
    public void registerRobot(
            Distance width,
            Distance length,
            Distance bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.robotPoseSupplier = poseSupplier;
        this.robotFieldSpeedsSupplier = fieldSpeedsSupplier;
        this.robotWidth = width.in(Meters);
        this.robotLength = length.in(Meters);
        this.bumperHeight = bumperHeight.in(Meters);
    }

    /**
     * To be called periodically
     * Will do nothing if sim is not running
     */
    public void updateSim() {
        if (!running) return;

        stepSim();
    }

    /**
     * Run the simulation forward 1 time step (0.02s)
     */
    public void stepSim() {
        for (int i = 0; i < subticks; i++) {
            for (Fuel fuel : fuels) {
                if (fuel.asleep) continue; // resting pile: nothing to integrate
                fuel.update(this.simulateAirResistance, this.subticks,
                        this.dragK, this.liftK, this.groundFriction);
                if (fuel.asleep) restingDirty = true; // it settled on this tick
            }

            handleFuelCollisions(fuels);

            if (robotPoseSupplier != null) {
                handleRobotCollisions(fuels);
                handleIntakes(fuels);
            }
        }

        logFuelsInFlight();
        if (loggingFreqHz >= 1.0 / PERIOD || loggingTimer.advanceIfElapsed(1.0 / loggingFreqHz)) {
            logFuels();
        }
    }

    /**
     * Adds a fuel onto the field
     * @param pos Position to spawn at
     * @param vel Initial velocity vector
     */
    public void spawnFuel(Translation3d pos, Translation3d vel) {
        makeRoom();
        fuels.add(new Fuel(pos, vel));
    }

    public void spawnFuel(Translation3d pos, Translation3d vel, double spinRadPerSec) {
        makeRoom();
        fuels.add(new Fuel(pos, vel, spinRadPerSec));
    }

    /**
     * The field holds a fixed amount of fuel, but nothing stops a preload or a reset from
     * putting more on it, and every extra ball is simulated forever. Past the cap the oldest
     * settled fuel is retired so a long session cannot keep getting slower.
     */
    protected void makeRoom() {
        if (fuels.size() < maxFuel) return;
        for (int i = 0; i < fuels.size(); i++) {
            if (fuels.get(i).asleep) {
                fuels.remove(i);
                restingDirty = true;
                return;
            }
        }
        fuels.remove(0);
    }

    /** Sets the most fuel allowed on the field at once. */
    public void setMaxFuel(int maxFuel) {
        this.maxFuel = maxFuel;
    }

    /**
     * Spawns a fuel onto the field with a specified launch velocity and angles, accounting for robot movement
     * @param launchVelocity Initial launch velocity
     * @param hoodAngle Hood angle where 0 is launching horizontally and 90 degrees is launching straight up
     * @param turretYaw <i>Robot-relative</i> turret yaw
     * @param launchHeight Height of the fuel to launch at. Make sure this is higher than your robot's bumper height, or else it will collide with your robot immediately.
     * @throws IllegalStateException if robot is not registered
     */
    public void launchFuel(LinearVelocity launchVelocity, Angle hoodAngle, Angle turretYaw, Distance launchHeight) {
        if (robotPoseSupplier == null || robotFieldSpeedsSupplier == null) {
            throw new IllegalStateException("Robot must be registered before launching fuel.");
        }

        Pose3d launchPose = new Pose3d(this.robotPoseSupplier.get())
                .plus(new Transform3d(new Translation3d(Meters.zero(), Meters.zero(), launchHeight), Rotation3d.kZero));
        ChassisSpeeds fieldSpeeds = this.robotFieldSpeedsSupplier.get();

        double horizontalVel = Math.cos(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);
        double verticalVel = Math.sin(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);
        double xVel = horizontalVel
                * Math.cos(
                        turretYaw.plus(launchPose.getRotation().getMeasureZ()).in(Radians));
        double yVel = horizontalVel
                * Math.sin(
                        turretYaw.plus(launchPose.getRotation().getMeasureZ()).in(Radians));

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        spawnFuel(launchPose.getTranslation(), new Translation3d(xVel, yVel, verticalVel));
    }

    protected void handleRobotCollision(Fuel fuel, Pose2d robot, Translation2d robotVel) {
        Translation2d relativePos = new Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
                .relativeTo(robot)
                .getTranslation();

        if (fuel.pos.getZ() > bumperHeight) return; // above bumpers
        double distanceToBottom = -FUEL_RADIUS - robotLength / 2 - relativePos.getX();
        double distanceToTop = -FUEL_RADIUS - robotLength / 2 + relativePos.getX();
        double distanceToRight = -FUEL_RADIUS - robotWidth / 2 - relativePos.getY();
        double distanceToLeft = -FUEL_RADIUS - robotWidth / 2 + relativePos.getY();

        // not inside robot
        if (distanceToBottom > 0 || distanceToTop > 0 || distanceToRight > 0 || distanceToLeft > 0) return;

        if (fuel.asleep) {
            if (robotVel.getNorm() < 0.05) return; // parked against it: leave it where it lies
            fuel.wake();
            restingDirty = true;
        }

        Translation2d posOffset;
        // find minimum distance to side and send corresponding collision response
        if ((distanceToBottom >= distanceToTop
                && distanceToBottom >= distanceToRight
                && distanceToBottom >= distanceToLeft)) {
            posOffset = new Translation2d(distanceToBottom, 0);
        } else if ((distanceToTop >= distanceToBottom
                && distanceToTop >= distanceToRight
                && distanceToTop >= distanceToLeft)) {
            posOffset = new Translation2d(-distanceToTop, 0);
        } else if ((distanceToRight >= distanceToBottom
                && distanceToRight >= distanceToTop
                && distanceToRight >= distanceToLeft)) {
            posOffset = new Translation2d(0, distanceToRight);
        } else {
            posOffset = new Translation2d(0, -distanceToLeft);
        }

        posOffset = posOffset.rotateBy(robot.getRotation());
        fuel.pos = fuel.pos.plus(new Translation3d(posOffset));
        Translation2d normal = posOffset.div(posOffset.getNorm());
        if (fuel.vel.toTranslation2d().dot(normal) < 0)
            fuel.addImpulse(
                    new Translation3d(normal.times(-fuel.vel.toTranslation2d().dot(normal) * (1 + ROBOT_COR))));
        if (robotVel.dot(normal) > 0) fuel.addImpulse(new Translation3d(normal.times(robotVel.dot(normal))));
    }

    protected void handleRobotCollisions(ArrayList<Fuel> fuels) {
        Pose2d robot = robotPoseSupplier.get();
        ChassisSpeeds speeds = robotFieldSpeedsSupplier.get();
        Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        for (Fuel fuel : fuels) {
            handleRobotCollision(fuel, robot, robotVel);
        }
    }

    protected void handleIntakes(ArrayList<Fuel> fuels) {
        Pose2d robot = robotPoseSupplier.get();
        for (SimIntake intake : intakes) {
            for (int i = 0; i < fuels.size(); i++) {
                if (intake.shouldIntake(fuels.get(i), robot)) {
                    restingDirty |= fuels.get(i).asleep;
                    fuels.remove(i);
                    i--;
                }
            }
        }
    }

    protected static final double BUMP_RAMP_START = 3.96;
    protected static final double BUMP_PEAK = 4.61;
    protected static final double BUMP_RAMP_END = 5.18;
    protected static final double BUMP_HEIGHT = 0.165;

    /**
     * Height of the floor at a point on the field: zero everywhere except the four bumps, which
     * are 15 degree ridges. Same geometry the fuel bounces off, so anything driving on it and
     * anything rolling over it agree.
     *
     * @param x field x in metres
     * @param y field y in metres
     * @return floor height in metres
     */
    public static double groundHeight(double x, double y) {
        boolean onBump = (y >= 1.57 && y <= FIELD_WIDTH / 2 - 0.60)
                || (y >= FIELD_WIDTH / 2 + 0.60 && y <= FIELD_WIDTH - 1.57);
        if (!onBump) return 0.0;
        double near = ridgeHeight(x);
        return near > 0.0 ? near : ridgeHeight(FIELD_LENGTH - x);
    }

    private static double ridgeHeight(double x) {
        if (x <= BUMP_RAMP_START || x >= BUMP_RAMP_END) return 0.0;
        return x < BUMP_PEAK
                ? BUMP_HEIGHT * (x - BUMP_RAMP_START) / (BUMP_PEAK - BUMP_RAMP_START)
                : BUMP_HEIGHT * (BUMP_RAMP_END - x) / (BUMP_RAMP_END - BUMP_PEAK);
    }

    protected static void fuelCollideRectangle(Fuel fuel, Translation3d start, Translation3d end) {
        if (fuel.pos.getZ() > end.getZ() + FUEL_RADIUS || fuel.pos.getZ() < start.getZ() - FUEL_RADIUS)
            return; // above rectangle
        double distanceToLeft = start.getX() - FUEL_RADIUS - fuel.pos.getX();
        double distanceToRight = fuel.pos.getX() - end.getX() - FUEL_RADIUS;
        double distanceToTop = fuel.pos.getY() - end.getY() - FUEL_RADIUS;
        double distanceToBottom = start.getY() - FUEL_RADIUS - fuel.pos.getY();

        // not inside hub
        if (distanceToLeft > 0 || distanceToRight > 0 || distanceToTop > 0 || distanceToBottom > 0) return;

        Translation2d collision;
        // find minimum distance to side and send corresponding collision response
        if (fuel.pos.getX() < start.getX()
                || (distanceToLeft >= distanceToRight
                        && distanceToLeft >= distanceToTop
                        && distanceToLeft >= distanceToBottom)) {
            collision = new Translation2d(distanceToLeft, 0);
        } else if (fuel.pos.getX() >= end.getX()
                || (distanceToRight >= distanceToLeft
                        && distanceToRight >= distanceToTop
                        && distanceToRight >= distanceToBottom)) {
            collision = new Translation2d(-distanceToRight, 0);
        } else if (fuel.pos.getY() > end.getY()
                || (distanceToTop >= distanceToLeft
                        && distanceToTop >= distanceToRight
                        && distanceToTop >= distanceToBottom)) {
            collision = new Translation2d(0, -distanceToTop);
        } else {
            collision = new Translation2d(0, distanceToBottom);
        }

        if (collision.getX() != 0) {
            fuel.pos = fuel.pos.plus(new Translation3d(collision));
            fuel.vel = fuel.vel.plus(new Translation3d(-(1 + FIELD_COR) * fuel.vel.getX(), 0, 0));
        } else if (collision.getY() != 0) {
            fuel.pos = fuel.pos.plus(new Translation3d(collision));
            fuel.vel = fuel.vel.plus(new Translation3d(0, -(1 + FIELD_COR) * fuel.vel.getY(), 0));
        }
    }

    /**
     * Registers an intake with the fuel simulator. This intake will remove fuel from the field based on the `ableToIntake` parameter.
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     * @param ableToIntake Should a return a boolean whether the intake is active
     * @param intakeCallback Function to call when a fuel is intaked
     */
    public void registerIntake(
            double xMin, double xMax, double yMin, double yMax, BooleanSupplier ableToIntake, Runnable intakeCallback) {
        intakes.add(new SimIntake(xMin, xMax, yMin, yMax, ableToIntake, intakeCallback));
    }

    /**
     * Registers an intake with the fuel simulator. This intake will remove fuel from the field based on the `ableToIntake` parameter.
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     * @param ableToIntake Should a return a boolean whether the intake is active
     */
    public void registerIntake(double xMin, double xMax, double yMin, double yMax, BooleanSupplier ableToIntake) {
        registerIntake(xMin, xMax, yMin, yMax, ableToIntake, () -> {});
    }

    /**
     * Registers an intake with the fuel simulator. This intake will always remove fuel from the field.
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     * @param intakeCallback Function to call when a fuel is intaked
     */
    public void registerIntake(double xMin, double xMax, double yMin, double yMax, Runnable intakeCallback) {
        registerIntake(xMin, xMax, yMin, yMax, () -> true, intakeCallback);
    }

    /**
     * Registers an intake with the fuel simulator. This intake will always remove fuel from the field.
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     */
    public void registerIntake(double xMin, double xMax, double yMin, double yMax) {
        registerIntake(xMin, xMax, yMin, yMax, () -> true, () -> {});
    }

    /**
     * Registers an intake with the fuel simulator. This intake will remove fuel from the field based on the `ableToIntake` parameter.
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     * @param ableToIntake Should a return a boolean whether the intake is active
     * @param intakeCallback Function to call when a fuel is intaked
     */
    public void registerIntake(
            Distance xMin,
            Distance xMax,
            Distance yMin,
            Distance yMax,
            BooleanSupplier ableToIntake,
            Runnable intakeCallback) {
        registerIntake(
                xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), ableToIntake, intakeCallback);
    }

    /**
     * Registers an intake with the fuel simulator. This intake will remove fuel from the field based on the `ableToIntake` parameter.
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     * @param ableToIntake Should a return a boolean whether the intake is active
     */
    public void registerIntake(
            Distance xMin, Distance xMax, Distance yMin, Distance yMax, BooleanSupplier ableToIntake) {
        registerIntake(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), ableToIntake);
    }

    /**
     * Registers an intake with the fuel simulator. This intake will always remove fuel from the field.
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     * @param intakeCallback Function to call when a fuel is intaked
     */
    public void registerIntake(Distance xMin, Distance xMax, Distance yMin, Distance yMax, Runnable intakeCallback) {
        registerIntake(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), intakeCallback);
    }

    /**
     * Registers an intake with the fuel simulator. This intake will always remove fuel from the field.
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     */
    public void registerIntake(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
        registerIntake(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
    }

    public static class Hub {
        public static final Hub BLUE_HUB =
                new Hub(new Translation2d(4.61, FIELD_WIDTH / 2), new Translation3d(5.3, FIELD_WIDTH / 2, 0.89), 1);
        public static final Hub RED_HUB = new Hub(
                new Translation2d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2),
                new Translation3d(FIELD_LENGTH - 5.3, FIELD_WIDTH / 2, 0.89),
                -1);

        protected static final double ENTRY_HEIGHT = 1.83;
        protected static final double ENTRY_RADIUS = 0.56;

        protected static final double SIDE = 1.2;
        protected static final double RETURN_TIME = 0.35;
        protected static final double RETURN_SPREAD = 0.6;

        protected static final double NET_HEIGHT_MAX = 3.057;
        protected static final double NET_HEIGHT_MIN = 1.5;
        protected static final double NET_OFFSET = SIDE / 2 + 0.261;
        protected static final double NET_WIDTH = 1.484;

        protected final Translation2d center;
        protected final Translation3d exit;
        protected final int exitVelXMult;

        protected int score = 0;

        protected Hub(Translation2d center, Translation3d exit, int exitVelXMult) {
            this.center = center;
            this.exit = exit;
            this.exitVelXMult = exitVelXMult;
        }

        protected void handleHubInteraction(Fuel fuel, int subticks) {
            if (didFuelScore(fuel, subticks)) {
                fuel.hubReturn = RETURN_TIME;
                fuel.hubEntry = fuel.pos;
                // Spread the returns across the chute. Landing every scored fuel on the same
                // point spawns them inside each other, and the overlap fix flings them apart.
                fuel.hubExit = exit.plus(new Translation3d(0, (Math.random() - 0.5) * RETURN_SPREAD, 0));
                fuel.hubExitVel = getDispersalVelocity();
                score++;
            }
        }

        protected boolean didFuelScore(Fuel fuel, int subticks) {
            return fuel.pos.toTranslation2d().getDistance(center) <= ENTRY_RADIUS
                    && fuel.pos.getZ() <= ENTRY_HEIGHT
                    && fuel.pos.minus(fuel.vel.times(PERIOD / subticks)).getZ() > ENTRY_HEIGHT;
        }

        protected Translation3d getDispersalVelocity() {
            return new Translation3d(exitVelXMult * (Math.random() + 0.1) * 1.5, Math.random() * 2 - 1, 0);
        }

        /**
         * Reset this hub's score to 0
         */
        public void resetScore() {
            score = 0;
        }

        /**
         * Get the current count of fuel scored in this hub
         * @return
         */
        public int getScore() {
            return score;
        }

        protected void fuelCollideSide(Fuel fuel) {
            fuelCollideRectangle(
                    fuel,
                    new Translation3d(center.getX() - SIDE / 2, center.getY() - SIDE / 2, 0),
                    new Translation3d(center.getX() + SIDE / 2, center.getY() + SIDE / 2, ENTRY_HEIGHT - 0.1));
        }

        protected double fuelHitNet(Fuel fuel) {
            if (fuel.pos.getZ() > NET_HEIGHT_MAX || fuel.pos.getZ() < NET_HEIGHT_MIN) return 0;
            if (fuel.pos.getY() > center.getY() + NET_WIDTH / 2 || fuel.pos.getY() < center.getY() - NET_WIDTH / 2)
                return 0;
            if (fuel.pos.getX() > center.getX() + NET_OFFSET * exitVelXMult) {
                return Math.max(0, center.getX() + NET_OFFSET * exitVelXMult - (fuel.pos.getX() - FUEL_RADIUS));
            } else {
                return Math.min(0, center.getX() + NET_OFFSET * exitVelXMult - (fuel.pos.getX() + FUEL_RADIUS));
            }
        }
    }

    protected class SimIntake {
        double xMin, xMax, yMin, yMax;
        BooleanSupplier ableToIntake;
        Runnable callback;

        protected SimIntake(
                double xMin,
                double xMax,
                double yMin,
                double yMax,
                BooleanSupplier ableToIntake,
                Runnable intakeCallback) {
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
            this.ableToIntake = ableToIntake;
            this.callback = intakeCallback;
        }

        protected boolean shouldIntake(Fuel fuel, Pose2d robotPose) {
            if (!ableToIntake.getAsBoolean() || fuel.pos.getZ() > bumperHeight) return false;

            Translation2d fuelRelativePos = new Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
                    .relativeTo(robotPose)
                    .getTranslation();

            boolean result = fuelRelativePos.getX() >= xMin
                    && fuelRelativePos.getX() <= xMax
                    && fuelRelativePos.getY() >= yMin
                    && fuelRelativePos.getY() <= yMax;
            if (result) {
                callback.run();
            }
            return result;
        }
    }
}
