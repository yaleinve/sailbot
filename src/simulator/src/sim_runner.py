#!/usr/bin/env python2.7
# sim_runner.py   Linc Berkley  Dec 17
# Implements a physics-based simulator to test the other nodes
# Adapated from SailsRudder simulator (written in Go) at https://github.com/acshi/acshi/tree/master/sailing

import rospy
import math, signal, pdb

import gpsCalc
from sails_rudder.msg import SailsRudderPos
from airmar.msg import AirmarData

import numpy as np

wa=0
wv=10
wd1,wd2=0,0

W_A=30
W_A_SIG=1.5
W_DA_V=0
W_D1=20
W_D2=20
W_R1=100
W_R2=5
W_DSIG1=1.0/W_R1
W_DSIG2=1.0/W_R2

def getFirstWind():
	global wa,wv,wd1,wd2
	wa=0
	wv=10
	wd1,wd2=0,0
	return Vector.from_polar(wv,wa)

def getNextWind():
	global wa,wv,wd1,wd2,W_A,W_DA_V
	wd1+=np.random.normal(loc=-wd1/W_R1,scale=W_DSIG1)
	wd2+=np.random.normal(loc=-wd2/W_R2,scale=W_DSIG2)
	W_A+=np.random.normal(scale=W_A_SIG)
	return Vector.from_polar(wv, (W_A+W_D1*wd1+W_D2*wd2+np.random.normal(loc=0,scale=12))%360)

class Simulator:
    def __init__(self):
        # Coordinate convention: x-axis is NORTH, y-axis is EAST. This makes clockwise rotations positive, with
        # reference angle north, as used elsewher in this project.

        self.dt = 0.01  # Physics timestep
        self.feedback_interval = .2  # Airmar feedback publication interval

        self.pub = None  # Publisher will be created in self.simulate

        self.wind = getNextWind()
        self.sail_adjust = 5  # Degrees to add to sail positions to account for slack

        # Boat properties in SI units
        self.pos = Vector.null()
        self.ang = Vector(0, 0, 180)  # Angular position
        self.v = Vector.null()
        self.w = Vector.null()  # Angular velocity

        # Positions requested by sails_rudder
        self.set_main = 0.0
        self.set_jib = 0.0
        self.set_rudder = 0.0

        # Actual sail positions (positive is clockwise when seen from above)
        self.main = 0.0
        self.jib = 0.0

    def simulate(self):
        rospy.init_node('simulator')
        rospy.Subscriber('/sails_rudder_pos', SailsRudderPos, self.sails_rudder_callback)
        self.pub = rospy.Publisher('/airmar_data', AirmarData, queue_size=10)
        rospy.loginfo("[simulator] All subscribed, simulator has started!")

        sim_rate = rospy.Rate(1 / self.dt)  # Hz
        step_num = 0
        # while not rospy.is_shutdown():
        while True:
            self.physics_step()
            if step_num % int(self.feedback_interval / self.dt) == 0:
                self.publish_feedback()
            step_num += 1
            sim_rate.sleep()

    def sails_rudder_callback(self, msg):
        # TODO: Add something to account for time to move servos
        # rospy.loginfo("[simulator] Received sails_rudder message " + str(msg))
        self.set_main = msg.mainPos + self.sail_adjust
        self.set_jib = msg.jibPos + self.sail_adjust
        self.set_rudder = msg.rudderPos + self.sail_adjust

    def physics_step(self):
    	self.wind=getNextWind()
        # Numerical integration procedure based on https://en.wikipedia.org/wiki/Verlet_integration#Velocity_Verlet
        self.update_sails()
        # These are called force and torque in calc_accelerations, so I'm not
        # sure if the inertia is included properly, but this is how the Go
        # version did it
        accel, alpha = self.calc_accelerations()
        self.v = self.v.add(accel.mult(self.dt / 2))
        self.w = self.w.add(alpha.mult(self.dt / 2))

        self.pos = self.pos.add(self.v.mult(self.dt))
        self.ang = self.ang.add(self.w.mult(self.dt))
        self.ang.normalize_angles()

        self.update_sails()
        accel_2, alpha_2 = self.calc_accelerations()
        self.v = self.v.add(accel_2.mult(self.dt / 2))
        self.w = self.w.add(alpha_2.mult(self.dt / 2))

    def publish_feedback(self):
        msg = AirmarData()
        msg.heading = self.ang.z
        msg.amrRoll = self.ang.y  # This will always be 0, so TODO: Implement roll properly -- not essential, but would be nice
        msg.apWndSpd, msg.apWndDir = self.rel_wind().to_polar()
        msg.sog, msg.cog = self.v.to_polar()
        msg.truWndSpd, msg.truWndDir = self.wind.to_polar()
        # Reasonably hoping that simulation distances never become large enough for spherical geometry to create serious problems
        msg.lat, msg.long = gpsCalc.gpsVectorOffset(0.0, 0.0, self.pos.xy_angle(), self.pos.mag())
        msg.lat += 41.256243
        msg.long += -72.850389
        # rospy.loginfo("[simulator] Publishing airmar message " + str(msg))
        self.pub.publish(msg)

    def update_sails(self):
        max_sail_pos = 90 + self.sail_adjust

        # Wind direction, expressed in sail position conventions (bow->rudder is 0; positive is clockwise when viewed
        # from above; -180 to 180). This is the "equilibrium position" the wind is trying to push the sails to.
        wind_rel_dir = angle_180_range(self.rel_wind().xy_angle() + 180)
        # Is the equilibrium position within the sail bounds?
        main_pos_possible = self.set_main >= wind_rel_dir >= -self.set_main
        jib_pos_possible = self.set_jib >= wind_rel_dir >= -self.set_jib
        # Reperesents directions sails will be pushed by wind -- if positive, sail will be pushed in postive direction
        main_off_wind = angle_180_range(self.main - wind_rel_dir)
        jib_off_wind = angle_180_range(self.jib - wind_rel_dir)

        if main_off_wind * (wind_rel_dir - self.set_main) > 0 and main_pos_possible:
            # If wind is pushing sail toward the equilibrium position, and that position is possible, the sail will swing to it
            self.main = wind_rel_dir
        else:
            # Otherwise, it will swing toward the max position that the wind is pushing it toward
            self.main = math.copysign(self.set_main, main_off_wind)
        if jib_off_wind * (wind_rel_dir - self.set_jib) > 0 and jib_pos_possible:
            # If wind is pushing sail toward the equilibrium position, and that position is possible, the sail will swing to it
            self.jib = wind_rel_dir
        else:
            # Otherwise, it will swing toward the max position that the wind is pushing it toward
            self.jib = math.copysign(self.set_jib, jib_off_wind)

    # Returns (linear acceleration), (angular acceleration)
    def calc_accelerations(self):
        # Numbers here are from original Go code -- they don't entirely make sense, but magnitudes aren't super important
        # to this simulation, and the drag/friction constants would be hard to calculate.
        # TODO: Might try to recalculate constants

        main_mast = Vector(0, -1, 9)
        main_rel_center = Vector(0, 5, 0)  # Relative to mast
        # Add 180 to sail position to adjust to angle convention
        main_center = Simulator.calc_part_pos(main_mast, main_rel_center, self.ang.z + 90, self.main)

        jib_mast = Vector(0, -7, 5)  # Jib doesn't actually have a mast, but it acts like it has one here
        jib_rel_center = Vector(0, 5, 0)
        jib_center = Simulator.calc_part_pos(jib_mast, jib_rel_center, self.ang.z + 90, self.jib)

        rudder_base = Vector(0, 6.5, 0)
        rudder_rel_center = Vector(0, 1, 0)
        rudder_center = Simulator.calc_part_pos(rudder_base, rudder_rel_center, self.ang.z + 90, self.set_rudder)

        keel_center = Vector(0, 0, -5)
        base_mass_center = Vector(0, 0, -1)
        # Component-multiplied by net torque to get angular acceleration, a 3x3 tensor would be more correct, but this
        # is easier and probably reasonably accurate
        inverse_moment_inertia = Vector(0, 0.01, 0.1)

        # These constants (unitless) encapsulate lift/drag/density coefficients with air, water, surface areas, etc...
        main_constant = 0.05
        jib_constant = 0.03
        keel_constant = 40.0  # We also give the keel credit for other lateral drag and friction
        rudder_constant = 10.0

        # Friction constants (dimensionless)
        axial_friction = 0.05
        angular_friction = 0.1
        forward_friction = 0.5

        grav_force = Vector(0, 0, 50)

        ap_wind_vec = self.ap_wind().neg()  # Direction wind is moving, not coming from
        main_force = Vector.from_polar(1, self.ang.z + self.main).normal().vector_proj(ap_wind_vec).mult(main_constant)
        jib_force = Vector.from_polar(1, self.ang.z + self.jib).normal().vector_proj(ap_wind_vec).mult(jib_constant)
        keel_force = Vector.from_polar(1, self.ang.z).normal().vector_proj(self.v.neg()).mult(keel_constant)
        axial_drag = Vector.from_polar(1, self.ang.z).normal().vector_proj(self.v.neg().comp_mult(self.v.comp_abs())).mult(axial_friction)
        forward_drag = self.v.neg().comp_mult(self.v.comp_abs()).mult(forward_friction)
        rudder_force = Vector.from_polar(1, self.ang.z + self.set_rudder).normal().vector_proj(self.v.neg()).mult(rudder_constant)

        main_torque = inverse_moment_inertia.comp_mult(main_center.cross(main_force))
        jib_torque = inverse_moment_inertia.comp_mult(jib_center.cross(jib_force))
        keel_torque = inverse_moment_inertia.comp_mult(keel_center.cross(keel_force))
        rudder_torque = inverse_moment_inertia.comp_mult(rudder_center.cross(rudder_force))

        angular_drag_torque = self.w.neg().comp_mult(self.w.comp_abs()).mult(angular_friction)
        mass_center = Vector.null()
        mass_center.x, mass_center.z = rotate(base_mass_center.x, base_mass_center.z, self.ang.y)
        gravity_torque = inverse_moment_inertia.comp_mult(mass_center.cross(grav_force))

        forces = main_force.add(jib_force).add(keel_force).add(rudder_force).add(axial_drag).add(forward_drag)
        torques = main_torque.add(jib_torque).add(keel_torque).add(rudder_torque).add(gravity_torque).add(angular_drag_torque)
        return forces, torques

    # Get apparent wind vector with compass direction
    # Unlike in sails_rudder and airmar, direction is not relative to heading
    # Not a field, because it would be a pain to relcalculate it whenever velocity is updated)
    def ap_wind(self):
        return self.wind.add(self.v)  # Add, rather than subtract, because "wind direction" is the direction wind is coming from

    # Apparent wind relative to heading, as used in airmar and sails_rudder
    def rel_wind(self):
        ap = self.ap_wind()
        return Vector.from_polar(ap.mag(), ap.xy_angle() - self.ang.z)

    # Calculates position of part in global referenc frame when both base and part are rotated
    @staticmethod
    def calc_part_pos(base_vec, part_vec, boat_heading, part_heading):
        part_rotated = part_vec.xy_rotate(part_heading)  # Rotated to reference frame of boat
        total_unrotated = base_vec.add(part_rotated)  # Total in reference frame of boat
        return total_unrotated.xy_rotate(boat_heading)


# All angle arguments and returns in degrees
class Vector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def null(cls):
        return cls(0.0, 0.0, 0.0)

    @classmethod
    def from_spherical(cls, rho, theta, phi):
        theta_r = math.radians(theta)
        phi_r = math.radians(phi)
        return cls(rho * math.sin(phi_r) * math.cos(theta_r),
                   rho * math.sin(phi_r) * math.sin(theta_r),
                   rho * math.cos(phi_r))

    @classmethod
    def from_polar(cls, r, theta):
        return cls.from_spherical(r, theta, 90)

    def mag(self):
        return math.sqrt(pow(self.x, 2) + pow(self.y, 2) + pow(self.z, 2))

    def xy_angle(self):
        return math.degrees(math.atan2(self.y, self.x))

    def to_polar(self):
        return self.mag(), self.xy_angle()

    # Should only be called on vectors that represent angles in degrees
    def normalize_angles(self):
        self.x, self.y, self.z = [a % 360 for a in (self.x, self.y, self.z)]

    def neg(self):
        return self.mult(-1)

    def comp_abs(self):
        return Vector(abs(self.x), abs(self.y), abs(self.z))

    def add(self, v):
        return Vector(self.x + v.x, self.y + v.y, self.z + v.z)

    def sub(self, v):
        return Vector(self.x - v.x, self.y - v.y, self.z - v.z)

    def mult(self, a):
        return Vector(self.x * a, self.y * a, self.z * a)

    def dot(self, v):
        return self.x * v.x + self.y * v.y + self.z * v.z

    def cross(self, v):
        return Vector(self.y * v.z - self.z * v.y, self.z * v.x - self.x * v.z, self.x * v.y - self.y * v.x)

    def comp_mult(self, v):
        return Vector(self.x * v.x, self.y * v.y, self.z * v.z)

    def scalar_proj(self, v):
        return self.dot(v) / self.mag()

    def vector_proj(self, v):
        return self.mult(self.dot(v)).mult(1 / pow(self.mag(), 2))

    def xy_rotate(self, a):
        new = Vector(0, 0, self.z)
        new.x, new.y = rotate(self.x, self.y, a)
        return new

    # Counterclockwise normal in 2D
    def normal(self):
        return Vector(-self.y, self.x, 0);

    def __str__(self):
        return str(self.x) + ', ' + str(self.y) + ', ' + str(self.z)

    def __repr__(self):
        return self.__str__()


# Return angle in -180 to 180 range that is equivalent to input angle
def angle_180_range(a):
    return ((a + 180) % 360) - 180


# Rotates in standard direction (from x-axis to y-axis, clockwise under our conventions)
def rotate(x, y, a):
    ar = math.radians(a)
    return (x * math.cos(ar) - y * math.sin(ar)), (x * math.sin(ar) + y * math.cos(ar))

def debug_signal_handler(signal, frame):
    pdb.set_trace()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, debug_signal_handler)
    sim = Simulator()
    sim.simulate()
