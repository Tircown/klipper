# Code for handling the kinematics of hybrid-corexz robots
# The hybrid-corexz kinematic is also known as Markforged kinematics
#
# Copyright (C) 2021  Fabrice Gallet <tircown@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, copy
import stepper, homing
from extras import idex_modes

class HybridCoreXZKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        printer_config = config.getsection('printer')
        # itersolve parameters
        self.rails = [ stepper.PrinterRail(config.getsection('stepper_x')),
                       stepper.LookupMultiRail(config.getsection('stepper_y')),
                       stepper.LookupMultiRail(config.getsection('stepper_z'))]
        self.rails[2].get_endstops()[0][0].add_stepper(
            self.rails[0].get_steppers()[0])
        self.rails[0].setup_itersolve('corexz_stepper_alloc', '-')
        self.rails[1].setup_itersolve('cartesian_stepper_alloc', 'y')
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', 'z')
        if config.has_section('dual_carriage'):
            self.printer.add_object("dual_carriage", self)
            dc_config = config.getsection('dual_carriage')
            # dummy for cartesian config users
            dc_config.getchoice('axis', {'x': 'x'}, default='x')
            # setup second dual carriage rail
            self.rails.append(stepper.PrinterRail(dc_config))
            self.rails[2].get_endstops()[0][0].add_stepper(
                self.rails[3].get_steppers()[0])
            self.rails[3].setup_itersolve('cartesian_stepper_alloc', 'z')
            dc_rail_0 = idex_modes.DualCarriagesRail(
                self.printer, self.rails[0], axis=0, active=True,
                stepper_alloc_active= ('corexz_stepper_alloc', '-'),
                stepper_alloc_inactive = ('hybrid_corexz_stepper_alloc', 'w'))
            dc_rail_1 = idex_modes.DualCarriagesRail(
                self.printer, self.rails[3], axis=0, active=False,
                stepper_alloc_active = ('corexz_stepper_alloc', '+'),
                stepper_alloc_inactive = ('cartesian_stepper_alloc', 'z'),
                stepper_alloc_reverse = ('hybrid_corexz_stepper_alloc', 'r'))
            self.dc_module = idex_modes.DualCarriages(self.printer,
                        dc_rail_0, dc_rail_1, axis=0)
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        self.printer.register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 3
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)
        # Setup stepper max halt velocity
        max_halt_velocity = toolhead.get_max_axis_halt()
        self.rails[0].set_max_jerk(max_halt_velocity, max_accel)
        self.rails[1].set_max_jerk(max_halt_velocity, max_accel)
        self.rails[2].set_max_jerk(
            min(max_halt_velocity, self.max_z_velocity), self.max_z_accel)
        if len(self.rails) == 4:
            self.rails[3].set_max_jerk(max_halt_velocity, max_accel)
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def calc_tag_position(self):
        pos = [rail.get_tag_position() for rail in self.rails]
        return [pos[0] - pos[2], pos[1], pos[2]]
    def update_limits(self, i, range):
        self.limits[i] = range
    def override_rail(self, i, rail):
        self.rails[i] = rail
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[2] = (1.0, -1.0)
    def _home_axis(self, homing_state, axis, rail):
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
    def force_home_axis(self, axis, rail):
        # Allows homing rail in the idex_mode module
        homing_state = homing.Homing(self.printer)
        self._home_axis(homing_state, axis, rail)
    def home(self, homing_state):
        for axis in homing_state.get_axes():
            if (hasattr(self, 'dc_module') and axis == 0):
                saved_status = self.dc_module.get_status();
                for i in [0,1]:
                    self.dc_module.toggle_active_dc_rail(i)
                    self._home_axis(homing_state, axis, self.rails[0])
                self.dc_module.recover_status(saved_status)
            else:
                self._home_axis(homing_state, axis, self.rails[axis])
    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
            'dual_carriage_status': self.dc_module.get_status()[mode]
        }

def load_kinematics(toolhead, config):
    return HybridCoreXZKinematics(toolhead, config)
