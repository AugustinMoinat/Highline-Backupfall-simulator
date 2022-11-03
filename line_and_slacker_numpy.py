from math import sqrt, fabs
import pygame
import pandas as pd
from random import randint
import pickle
import numpy as np


class Element:
    # This class is a master class for the classes: Anchor, Segment, Split, and free end.
    # Each subbclass needs to have functions :
    # compute_acceleration
    # compute_speed
    # compute_position
    def move(self, dt):  # This is where RK4 should get implemented
        self.compute_acceleration()
        self.compute_speed(dt)
        self.compute_position(dt)

    def draw(self, win, scale):
        pygame.draw.circle(win, (255, max(255*(1-self.t/self.maxt), 0), max(255*(1-self.t/self.maxt), 0)),
                           (self.x*scale, self.y*scale), scale)


class Anchor(Element):
    # Nothing to see here
    def compute_acceleration(self):
        pass

    def compute_speed(self, dt):
        pass

    def compute_position(self, dt):
        pass


class Webbing:
    def __init__(self, lw, stretch, force, d):
        # linear weight, stretch amount (10% -> 0.1), force for that stretch (N), drag
        self.lw = lw
        self.k = force/stretch
        self.d = d

    def describe(self):
        return f'lw:{self.lw * 1000} g/m_sc:{self.k/1000:.2f}kN/m_dc:{self.d}'


class Segment(Element):
    def __init__(self, webbing, length, num_points, start_point, end_point, maxt=10000, mint=0):
        self.webbing = webbing
        self.len = length
        self.num_points = num_points  # number of points
        self.dl = length/(num_points + 1)
        self.dw = webbing.lw * self.dl
        self.k = webbing.k
        self.d = webbing.d
        self.maxt = maxt  # for color scale
        self.mint = mint
        self.points = np.linspace(start_point, end_point, num_points+2)
        self.speeds = np.zeros([num_points, 2])
        self.accelerations = np.zeros([num_points, 2])
        self.tensions = np.zeros(num_points + 1)
        self.tensions_coordinates = np.zeros([num_points + 1, 2])

    def compute_acceleration(self):
        # Initialize at 0
        self.accelerations = np.zeros([self.num_points, 2])
        # Compute the different forces
        self.elastic_force()
        self.gravity_force()
        self.drag_force()

    def elastic_force(self):
        diff_points = np.diff(self.points, axis=0)
        dist_points = np.sqrt(np.sum(np.square(diff_points), axis=1, keepdims=True))
        self.tensions = self.k * np.fmax(dist_points - self.dl, 0) / self.dl
        self.tensions_coordinates = np.divide(np.multiply(diff_points, self.tensions), dist_points)
        self.accelerations += np.diff(self.tensions_coordinates, axis=0) / self.dw

    def gravity_force(self):
        self.accelerations += [0, 9.81]

    def drag_force(self):
        self.accelerations -= self.d * self.dl * self.speeds * np.abs(self.speeds) / self.dw

    def compute_speed(self, dt):
        self.speeds += self.accelerations * dt

    def compute_position(self, dt):
        self.points += np.append(np.append([[0, 0]], self.speeds, axis=0), [[0, 0]], axis=0) * dt

    def describe(self):
        return f':{self.len}m of ' + self.webbing.describe()

    def draw(self, win, scale):
        colors = 255 * np.fmin(np.fmax((self.maxt - self.tensions) / (self.maxt - self.mint), [0]), [1])
        for i in range(self.num_points+1):
            pygame.draw.line(win, (255, colors[i], colors[i]),(self.points[i,0] * scale, self.points[i,1] * scale),
                             (self.points[i+1, 0] * scale, self.points[i+1, 1] * scale), 3)


class CutEnd(Element):
    def __init__(self, segment, start, w, d):
        self.w = w  # weight
        self.d = d  # drag
        self.segment = segment
        self.start = start  # bool (start vs end)
        if start:
            self.position = segment.points[0]
        else:
            self.position = segment.points[-1]
        self.acceleration = [0,0]
        self.speed = [0,0]

    def compute_acceleration(self):
        self.acceleration = [0, 9.81]
        if self.start:
            self.acceleration += self.segment.tensions_coordinates[0] / self.w
        else:
            self.acceleration -= self.segment.tensions_coordinates[-1] / self.w
        self.acceleration -= self.speed * np.fabs(self.speed) * self.d / self.w

    def compute_speed(self, dt):
        self.speed += self.acceleration * dt

    def compute_position(self,dt):
        self.position += self.speed * dt
        if self.start:
            self.segment.points[0] = self.position
        else:
            self.segment.points[-1] = self.position


class WebbingUnit(Element):
    def __init__(self, x, y, w, l, k, d, maxt=12750, is_cut=False):
        # position, weight, natural length, stretch constant,drag, max tension (for color scale)
        self.x = x
        self.y = y
        self.vx = 0  # horizontal speed
        self.vy = 0  # vertical speed
        self.ax = 0  # horizontal acceleration
        self.ay = 0  # vertical acceleration
        self.w = w
        self.l = l
        self.k = k
        self.xp = x - l/2  # connection point with previous element
        self.xn = x + l/2  # connection point with next element
        self.yp = y
        self.yn = y
        self.d = d
        self.t = 0
        self.maxt = maxt
        self.is_cut = is_cut

    def draw(self, win, scale):
        if not self.is_cut:
            pygame.draw.line(win, (255, max(255*(1-self.t/self.maxt),0), max(255*(1-self.t/self.maxt), 0)),
                             (self.xp*scale, self.yp*scale), (self.xn*scale, self.yn*scale), 3)

    def connection_next(self, next):
        # the connection point is in the middle
        self.xn = (self.x + next.x) / 2
        self.yn = (self.y + next.y) / 2
        next.xp = self.xn
        next.yp = self.yn
        # except in the cases where one of the two is cut, then there is no connection, use default position
        if self.is_cut:
            next.xp = next.x
            next.yp = next.y + next.l / 2
        if next.is_cut:
            self.xn = self.x
            self.yn = self.y + self.l / 2

    def acceleration(self):
        # forces that apply: elasticity and gravity
        g = 9.81 * self.w
        dn = sqrt(pow((self.x - self.xn), 2) + pow((self.y - self.yn), 2))
        dp = sqrt(pow((self.x - self.xp), 2) + pow((self.y - self.yp), 2))
        Fn = -self.k * max((dn - self.l/2), 0) * 2 / self.l
        Fp = -self.k * max((dp - self.l/2), 0) * 2 / self.l
        self.t = max(fabs(Fn), fabs(Fp))
        if self.is_cut:
            self.t = 0
        Fy = Fn * (self.y - self.yn)/dn + Fp * (self.y - self.yp)/dp + g - fabs(self.vy) * self.vy * self.d * (dn+dp)
        Fx = Fn * (self.x - self.xn)/dn + Fp * (self.x - self.xp)/dp - fabs(self.vx) * self.vx * self.d * (dn+dp)
        self.ax = Fx / self.w
        self.ay = Fy / self.w
        return self.t

    def copy_unit(self, is_cut=False):
        if is_cut:
            k = 0
        else:
            k = self.k
        copy = WebbingUnit(self.x,self.y,self.w,self.l,k,self.d,self.maxt, is_cut)
        copy.vx = self.vx
        copy.vy = self.vy
        copy.ax = self.ax
        copy.ay = self.ay
        copy.xp = self.xp
        copy.xn = self.xn
        copy.yp = self.yp
        copy.yn = self.yn
        copy.t = self.t
        return copy


class Split(Element):
    def __init__(self, x, y, w, lpm, lpb, lnm, lnb, kpm, kpb, knm, knb, d, maxt=12750):
        # position, weight, natural length, stretch constant for main and back-up, previous and next,
        # drag, max tension (for color scale)
        self.x = x
        self.y = y
        self.vx = 0  # horizontal speed
        self.vy = 0  # vertical speed
        self.w = w
        self.lpm = lpm
        self.lpb = lpb
        self.lnm = lnm
        self.lnb = lnb
        self.kpm = kpm
        self.kpb = kpb
        self.knm = knm
        self.knb = knb
        self.ax = 0  # horizontal acceleration
        self.ay = 0  # vertical acceleration
        self.xpm = x  # connection point with previous mainline element
        self.xnm = x  # connection point with next mainline element
        self.ypm = y
        self.ynm = y
        self.xpb = x  # connection point with previous backup element
        self.xnb = x  # connection point with next backup element
        self.ypb = y
        self.ynb = y
        self.d = d
        self.t = 0
        self.maxt = maxt

    def connection_point(self, previous_main, next_main, previous_backup, next_backup):
        # Connection point with previous and next WebbingUnit elements, with main and back-up
        self.xpm = (self.x + previous_main.x)/2
        self.ypm = (self.y + previous_main.y)/2
        previous_main.xn = self.xpm
        previous_main.yn = self.ypm
        self.xpb = (self.x + previous_backup.x) / 2
        self.ypb = (self.y + previous_backup.y) / 2
        previous_backup.xn = self.xpb
        previous_backup.yn = self.ypb
        if previous_main.is_cut:
            self.xpb = self.x
            self.ypb = self.y + self.lpb / 2
        self.xnm = (self.x + next_main.x) / 2
        self.ynm = (self.y + next_main.y) / 2
        next_main.xp = self.xnm
        next_main.yp = self.ynm
        self.xnb = (self.x + next_backup.x) / 2
        self.ynb = (self.y + next_backup.y) / 2
        next_backup.xp = self.xnb
        next_backup.yp = self.ynb
        if next_main.is_cut:
            self.xnb = self.x
            self.ynb = self.y + self.lnb / 2

    def acceleration(self):
        # forces that apply: elasticity and gravity
        g = 9.81 * self.w
        dnm = sqrt(pow((self.x - self.xnm), 2) + pow((self.y - self.ynm), 2))
        dpm = sqrt(pow((self.x - self.xpm), 2) + pow((self.y - self.ypm), 2))
        dnb = sqrt(pow((self.x - self.xnb), 2) + pow((self.y - self.ynb), 2))
        dpb = sqrt(pow((self.x - self.xpb), 2) + pow((self.y - self.ypb), 2))
        Fnm = -self.knm * max((dnm - self.lnm), 0) / self.lnm
        Fpm = -self.kpm * max((dpm - self.lpm), 0) / self.lpm
        Fnb = -self.knb * max((dnb - self.lnb), 0) / self.lnb
        Fpb = -self.kpb * max((dpb - self.lpb), 0) / self.lpb
        self.t = max(fabs(Fnm), fabs(Fpm), fabs(Fnb), fabs(Fpb))
        Fmy = Fnm * (self.y - self.ynm) / dnm + Fpm * (self.y - self.ypm) / dpm
        Fby = Fnb * (self.y - self.ynb) / dnb + Fpb * (self.y - self.ypb) / dpb
        Fmx = Fnm * (self.x - self.xnm) / dnm + Fpm * (self.x - self.xpm) / dpm
        Fbx = Fnb * (self.x - self.xnb) / dnb + Fpb * (self.x - self.xpb) / dpb
        Fy = Fmy + Fby + g - fabs(self.vy) * self.vy * self.d * (dnm+dpm+dnb+dpb)
        Fx = Fmx + Fbx - fabs(self.vx) * self.vx * self.d * (dnm+dpm+dnb+dpb)
        self.ax = Fx / self.w
        self.ay = Fy / self.w
        return self.t

    def draw(self, win, scale):
        pygame.draw.line(win, (255, max(255*(1-self.t/self.maxt),0), max(255*(1-self.t/self.maxt),0)),
                         (self.x*scale,self.y*scale), (self.xnm*scale,self.ynm*scale), 3)
        pygame.draw.line(win, (255, max(255 * (1 - self.t / self.maxt), 0), max(255 * (1 - self.t / self.maxt), 0)),
                         (self.x * scale, self.y * scale), (self.xnb * scale, self.ynb * scale), 3)
        pygame.draw.line(win, (255, max(255 * (1 - self.t / self.maxt), 0), max(255 * (1 - self.t / self.maxt), 0)),
                         (self.x * scale, self.y * scale), (self.xpm * scale, self.ypm * scale), 3)
        pygame.draw.line(win, (255, max(255 * (1 - self.t / self.maxt), 0), max(255 * (1 - self.t / self.maxt), 0)),
                         (self.x * scale, self.y * scale), (self.xpb * scale, self.ypb * scale), 3)


class Spot:
    def __init__(self, g, h):
        # gap length, height
        self.g = g
        self.h = h
        self.anchor1 = Anchor(0, h)
        self.anchor2 = Anchor(g, h)

    def describe(self):
        return f'length: {self.g} m, height: {self.h} m'


class Rig:
    def __init__(self, setup, spot, maxt=12750):
        # setup: list of Segments not empty
        self.segments = setup
        self.spot = spot
        self.maxt = maxt
        # calculate set-up mainline length and number of segments
        self.main_length = 0
        self.num_segment = 0
        for segment in self.segments:
            self.main_length += segment.main_l
            self.num_segment += 1
        ratio = spot.g/self.main_length  # for original placement of  the line
        self.mains = []  # list of lists of WebbingUnit
        self.backups = []  # list of lists of WebbingUnit
        self.splits = []  # list of Split
        self.tension_main = []
        self.i_tension_main = []
        self.tension_backup = []
        self.i_tension_backup = []
        self.tension_split = []
        self.i_tension_split = []

        # Compute the WebbingUnit and Split
        if self.num_segment == 0:
            raise ValueError
        elif self.num_segment == 1:
            # single segment goes anchor to anchor
            segment = self.segments[0]
            dx = self.spot.g/segment.np  # horizontal space between each WebbingUnit
            dlm = segment.main_l/segment.np  # length of each WebbingUnit for mainline
            dlb = segment.backup_l/segment.np  # length of each WebbingUnit for backup
            main = []  # contains the WebbingUnit of the mainline
            back = []  # contains the WebbingUnit of the backup
            for i in range(segment.np):
                main.append(WebbingUnit((i+0.5)*dx, self.spot.h, segment.main.lw*dlm,
                                        dlm, segment.main.k, segment.main.d, maxt))
                back.append(WebbingUnit((i+0.5)*dx, self.spot.h, segment.backup.lw*dlb,
                                        dlb, segment.backup.k, segment.backup.d, maxt))
            self.mains.append(main)
            self.tension_main.append(0)
            self.i_tension_main.append(0)
            self.backups.append(back)
            self.tension_backup.append(0)
            self.i_tension_backup.append(0)
        else:
            segment = self.segments[0]
            segment_length = segment.main_l*ratio  # stretch to fit the gap
            dx = segment_length/(segment.np+0.5)  # half a segment is for the split
            dlm = segment.main_l/(segment.np+0.5)
            dlb = segment.backup_l/(segment.np+0.5)
            km = segment.main.k
            kb = segment.backup.k
            dm = segment.main.d
            db = segment.backup.d
            main = []
            back = []
            # Create WebbingUnit for the first segment
            for i in range(segment.np):
                main.append(WebbingUnit((i+0.5)*dx, self.spot.h, segment.main.lw*dlm, dlm, km, dm, maxt))
                back.append(WebbingUnit((i+0.5)*dx, self.spot.h, segment.backup.lw*dlb, dlb, kb, db, maxt))
            self.mains.append(main)
            self.tension_main.append(0)
            self.i_tension_main.append(0)
            self.backups.append(back)
            self.tension_backup.append(0)
            self.i_tension_backup.append(0)
            split_weight = 0.5*(segment.main.lw*dlm+segment.backup.lw*dlb)
            zero_point = segment_length  # Base point for next split and segment

            # Treat all intermediate segments
            for i in range(1, self.num_segment-1):
                segment = self.segments[i]
                segment_length = segment.main_l * ratio
                dx = segment_length / (segment.np + 1)  # intermediate segments have 2 half lengths for splits
                dlm2 = segment.main_l / (segment.np + 1)
                dlb2 = segment.backup_l / (segment.np + 1)
                # Get constants to create the split, keep previous ones
                km2 = segment.main.k
                kb2 = segment.backup.k
                dm2 = segment.main.d
                db2 = segment.backup.d
                split_weight2 = 0.5*(segment.main.lw*dlm+segment.backup.lw*dlb)
                # create the Split
                self.splits.append(Split(zero_point, self.spot.h, split_weight+split_weight2,
                                         dlm/2, dlb/2, dlm2/2, dlb2/2, km, kb, km2, kb2, dm+db+dm2+db2, maxt))
                self.tension_split.append(0)
                self.i_tension_split.append(0)
                # update constants
                split_weight = split_weight2
                dlm = dlm2
                dlb = dlb2
                km = km2
                kb = kb2
                dm = dm2
                db = db2
                main = []
                back = []
                # Create the webbing units
                for j in range(segment.np):
                    main.append(WebbingUnit(zero_point + (j + 1) * dx, self.spot.h, segment.main.lw * dlm,
                                            dlm, km, dm, maxt))
                    back.append(WebbingUnit(zero_point + (j + 1) * dx, self.spot.h, segment.backup.lw * dlb,
                                            dlb, kb, db, maxt))
                self.mains.append(main)
                self.tension_main.append(0)
                self.i_tension_main.append(0)
                self.backups.append(back)
                self.tension_backup.append(0)
                self.i_tension_backup.append(0)
                # move reference point
                zero_point += segment_length

            # Create last Split and last segment of WebbingUnit,
            segment = self.segments[-1]
            segment_length = segment.main_l * ratio
            dx = segment_length / (segment.np + 0.5)  # half an extra length for the split
            dlm2 = segment.main_l / (segment.np + 0.5)
            dlb2 = segment.backup_l / (segment.np + 0.5)
            # Get constants to create the split, keep previous ones
            km2 = segment.main.k
            kb2 = segment.backup.k
            dm2 = segment.main.d
            db2 = segment.backup.d
            split_weight2 = 0.5 * (segment.main.lw * dlm + segment.backup.lw * dlb)
            # create the Split
            self.splits.append(Split(zero_point, self.spot.h, split_weight + split_weight2,
                                     dlm/2, dlb/2, dlm2/2, dlb2/2, km, kb, km2, kb2, dm + db + dm2 + db2, maxt))
            self.tension_split.append(0)
            self.i_tension_split.append(0)
            # Update the constants
            dlm = dlm2
            dlb = dlb2
            km = km2
            kb = kb2
            dm = dm2
            db = db2
            main = []
            back = []
            # Create the last segments of WebbingUnit
            for j in range(segment.np):
                main.append(WebbingUnit(zero_point + (j + 1) * dx, self.spot.h, segment.main.lw * dlm,
                                        dlm, km, dm, maxt))
                back.append(WebbingUnit(zero_point + (j + 1) * dx, self.spot.h, segment.backup.lw * dlb,
                                        dlb, kb, db, maxt))
            self.mains.append(main)
            self.tension_main.append(0)
            self.i_tension_main.append(0)
            self.backups.append(back)
            self.tension_backup.append(0)
            self.i_tension_backup.append(0)

    def connections(self):
        # updates all connection points
        # Connection with first anchor, for main and back-up first element of first segment
        self.spot.anchor1.start_connection(self.mains[0][0])
        self.spot.anchor1.start_connection(self.backups[0][0])
        # Connection of each (except for last) WebbingUnit with next one for first segment
        for j in range(self.segments[0].np-1):
            self.mains[0][j].connection_next(self.mains[0][j+1])
            self.backups[0][j].connection_next(self.backups[0][j + 1])
        # Run through all segments except first and last
        for i in range(1, self.num_segment-1):
            # Connection of split with previous and next, main and backup
            self.splits[i-1].connection_point(self.mains[i-1][-1], self.mains[i][0],
                                              self.backups[i-1][-1], self.backups[i][0])
            # Connection of each (except for last) WebbingUnit with next one for this segment
            for j in range(self.segments[i].np - 1):
                self.mains[i][j].connection_next(self.mains[i][j + 1])
                self.backups[i][j].connection_next(self.backups[i][j + 1])
        # connection for last split, with previous and next, main and backup
        if self.num_segment > 1:
            self.splits[-1].connection_point(self.mains[-2][-1], self.mains[-1][0],
                                             self.backups[-2][-1], self.backups[-1][0])
            # Connection of each (except for last) WebbingUnit with next one for the last segment
            for j in range(self.segments[-1].np - 1):
                self.mains[-1][j].connection_next(self.mains[-1][j + 1])
                self.backups[-1][j].connection_next(self.backups[-1][j + 1])
        # connection of anchor2 with last WebbingUnit of main and backup
        self.spot.anchor2.end_connection(self.mains[-1][-1])
        self.spot.anchor2.end_connection(self.backups[-1][-1])

    def get_tension(self):
        tension_split = []
        for i,t in enumerate(self.i_tension_split):
            tension_split.append(t)
            self.i_tension_split[i] = 0
        tension_main = []
        for i,t in enumerate(self.i_tension_main):
            tension_main.append(t)
            self.i_tension_main[i] = 0
        tension_backup = []
        for i, t in enumerate(self.i_tension_backup):
            tension_backup.append(t)
            self.i_tension_backup[i] = 0
        return tension_split, tension_main, tension_backup

    def draw(self, win, scale):
        for split in self.splits:
            split.draw(win, scale)
        for main in self.mains:
            for unit in main:
                unit.draw(win, scale)
        for backup in self.backups:
            for unit in backup:
                unit.draw(win, scale)

    def acceleration(self):
        for i, split in enumerate(self.splits):
            tension = split.acceleration()
            self.tension_split[i] = tension
            self.i_tension_split[i] = max(self.i_tension_split[i], tension)
        for i, main in enumerate(self.mains):
            tension = 0
            for unit in main:
                tension = max(tension, unit.acceleration())
            self.tension_main[i] = tension
            self.i_tension_main[i] = max(self.i_tension_main[i], tension)
        for i, backup in enumerate(self.backups):
            tension = 0
            for unit in backup:
                tension = max(tension, unit.acceleration())
            self.tension_backup[i] = tension
            self.i_tension_backup[i] = max(self.i_tension_backup[i], tension)

    def move(self,dt):
        for split in self.splits:
            split.move(dt)
        for main in self.mains:
            for unit in main:
                unit.move(dt)
        for backup in self.backups:
            for unit in backup:
                unit.move(dt)

    def run(self, t):
        self.connections()
        self.acceleration()
        self.move(t)

    def runs(self, t, steps):
        dt = t/steps
        for ii in range(steps):
            self.run(dt)

    def backup_fall(self, segment, unit):
        self.mains[segment][unit].is_cut = True

    def describe(self):
        description = ['Spot', self.spot.describe()]
        for i, segment in enumerate(self.segments):
            description.append(f'Section {i}')
            description.append(segment.describe())
        return description


class Slackliner:
    def __init__(self, leg_length, weight, leash_length, power, rig, segment=0, unit=0):
        self.leg_length = leg_length
        self.weight = weight
        self.leash_length = leash_length
        self.power = power  # greater than 1
        self.rig = rig  # a Rig
        self.segment = segment  # position on the line is given by segment, unit
        self.unit = unit
        self.standing = True  # Is the slackliner standing or falling?
        self.y = min(rig.mains[segment][unit].y, rig.backups[segment][unit].y) - leg_length
        self.vy = 0
        self.legs_bent = False
        self.backupfall = False
        self.backupfall_segment = -1
        self.leash_tension = 0
        self.leash_recorded = 0

    def move_left(self):
        y1 = self.rig.mains[self.segment][self.unit].y
        if self.unit == 0:
            if self.segment != 0:
                self.segment -= 1
                self.unit = self.rig.segments[self.segment].np - 1
        else:
            self.unit -= 1
        y2 = self.rig.mains[self.segment][self.unit].y
        self.y += y2 - y1

    def move_right(self):
        y1 = self.rig.mains[self.segment][self.unit].y
        if self.unit < self.rig.segments[self.segment].np -1:
            self.unit += 1
        elif self.segment < self.rig.num_segment - 1:
            self.segment += 1
            self.unit = 0
        y2 = self.rig.mains[self.segment][self.unit].y
        self.y += y2 - y1

    def fall(self):
        self.standing = False

    def bend(self):
        self.legs_bent = True
        if self.standing and self.y - self.leg_length / 2 < self.rig.mains[self.segment][self.unit].y:
            self.y = self.rig.mains[self.segment][self.unit].y - self.leg_length / 2

    def move(self, dt):
        # calculates physics
        seg = self.segment
        unit = self.unit
        g = 9.81 * self.weight
        f = 0
        self.leash_tension = 0
        # the legs can only bend down to 1/4 of their length
        if self.y + self.leg_length / 4 > self.rig.mains[seg][unit].y:
            self.standing = False
        if self.standing:
            # if the legs are bent, change leg length in the following computation
            if self.legs_bent:
                legs_length = self.leg_length / 2
            else:
                legs_length = self.leg_length
            # if the feet are in contact with the line, push with the available power
            if self.y + legs_length > self.rig.mains[seg][unit].y:
                f = - 9.81 * self.power * self.weight
                # force apply to the main line
                self.rig.mains[seg][unit].ay -= f / self.rig.mains[seg][unit].w
            if self.y + legs_length > self.rig.backups[seg][unit].y:
                f = - 9.81 * self.power * self.weight
                # force apply to the backup line
                self.rig.backups[seg][unit].ay -= f / self.rig.mains[seg][unit].w
        if self.backupfall_segment != self.segment and self.y - self.leash_length > self.rig.mains[seg][unit].y:
            # hanging in the leash but not when there is already a backup  fall
            f = -(10000/0.25) * (self.y - self.leash_length - self.rig.mains[seg][unit].y)/self.leash_length
            self.leash_tension = max(fabs(f), self.leash_tension)
            self.rig.mains[seg][unit].ay -= f / self.rig.mains[seg][unit].w
        if self.y - self.leash_length > self.rig.backups[seg][unit].y:
            f = -(10000 / 0.25) * (self.y - self.leash_length - self.rig.backups[seg][unit].y) / self.leash_length
            self.leash_tension = max(fabs(f), self.leash_tension)
            self.rig.backups[seg][unit].ay -= f / self.rig.backups[seg][unit].w
        ay = (f+g)/self.weight - 0.1 * self.vy
        self.vy += ay * dt
        self.y += self.vy * dt
        self.leash_recorded = max(self.leash_tension, self.leash_recorded)

    def draw(self, win, scale):
        color = (0, 0, 255)  # blue
        if not self.backupfall or self.backupfall_segment != self.segment:
            pygame.draw.circle(win, color, (self.rig.mains[self.segment][self.unit].x * scale, self.y * scale), 5)
        else:
            pygame.draw.circle(win, color, (self.rig.backups[self.segment][self.unit].x * scale, self.y * scale), 5)
        self.rig.draw(win, scale)

    def get_tension(self):
        tension = self.leash_recorded
        self.leash_recorded = 0
        return tension

    def run(self, dt):
        self.rig.connections()
        self.rig.acceleration()
        self.move(dt)
        self.rig.move(dt)

    def runs(self, t, steps):
        dt = t/steps
        for ii in range(steps):
            self.run(dt)

    def backup_fall(self, segment, unit=-1):
        if segment < self.rig.num_segment:
            self.backupfall = True
            self.backupfall_segment = segment
            if unit == -1:
                unit = randint(0, self.rig.segments[segment].np-1)
            self.rig.backup_fall(segment, unit)
            self.fall()


def draw(win, augustin, scale):
    # Graphics update
    pygame.display.update()
    win.fill((0, 0, 0))
    augustin.draw(win, scale)


def play(augustin, line, scale, max_steps, dt, steps, folder):
    # Graphics set-up
    win = pygame.display.set_mode((1280, 720))

    # Create recording lists
    leash_tension = []
    time = []
    back_up_fall = []
    main_tension_h = [[] for i in range(line.num_segment)]
    backup_tension_h = [[] for i in range(line.num_segment)]
    splits_tension_h = [[] for i in range(line.num_segment - 1)]
    max_force = []
    height = []

    run = True

    t = 0  # initialization of time
    test_time = 0  # Initialize the number of steps

    while run and test_time != max_steps:
        test_time += 1
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

        keys = pygame.key.get_pressed()  # get keys

        # Movements of augustin
        if keys[pygame.K_ESCAPE] or keys[pygame.K_BACKSPACE]:
            run = False
        if keys[pygame.K_LEFT]:
            augustin.move_left()
        if keys[pygame.K_RIGHT]:
            augustin.move_right()
        augustin.legs_bent = keys[pygame.K_DOWN]
        if keys[pygame.K_SPACE]:
            augustin.fall()

        # Save state
        if keys[pygame.K_RETURN]:
            out_file = open(folder+'save_state.pk', mode='w+b')

            pickle.dump(augustin, out_file)

            out_file.close()

        # Trigger the Back-up fall
        if keys[pygame.K_F1] and not augustin.backupfall:
            augustin.backup_fall(0)
        if keys[pygame.K_F2] and not augustin.backupfall:
            augustin.backup_fall(1)
        if keys[pygame.K_F3] and not augustin.backupfall:
            augustin.backup_fall(2)
        if keys[pygame.K_F4] and not augustin.backupfall:
            augustin.backup_fall(3)
        if keys[pygame.K_F5] and not augustin.backupfall:
            augustin.backup_fall(4)
        if keys[pygame.K_F6] and not augustin.backupfall:
            augustin.backup_fall(5)
        if keys[pygame.K_F7] and not augustin.backupfall:
            augustin.backup_fall(6)
        if keys[pygame.K_F8] and not augustin.backupfall:
            augustin.backup_fall(7)
        if keys[pygame.K_F9] and not augustin.backupfall:
            augustin.backup_fall(8)
        if keys[pygame.K_F10] and not augustin.backupfall:
            augustin.backup_fall(9)
        if keys[pygame.K_F11] and not augustin.backupfall:
            augustin.backup_fall(10)
        if keys[pygame.K_F12] and not augustin.backupfall:
            augustin.backup_fall(11)
        augustin.runs(dt, steps)
        t += dt
        time.append(t)
        leash_tension.append(augustin.get_tension()/1000)
        height.append( -augustin.y)
        back_up_fall.append(augustin.backupfall_segment)
        split_tension, main_tension, backup_tension = augustin.rig.get_tension()
        for i in range(line.num_segment):
            if i < line.num_segment - 1:
                splits_tension_h[i].append(split_tension[i]/1000)
            main_tension_h[i].append(main_tension[i]/1000)
            backup_tension_h[i].append(backup_tension[i]/1000)
        if line.num_segment > 1:
            max_force.append(max(max(main_tension), max(split_tension), max(backup_tension))/1000)
        else:
            max_force.append(max(max(main_tension), max(backup_tension))/1000)
        draw(win, augustin, scale)

    split_dict = {'split ' + str(i): splits_tension_h[i] for i in range(line.num_segment - 1)}
    main_dict = {'main ' + str(i): main_tension_h[i] for i in range(line.num_segment)}
    backup_dict = {'backup ' + str(i): backup_tension_h[i] for i in range(line.num_segment)}
    dico = {'time': time, 'leash': leash_tension, 'height': height, 'backup_fall': back_up_fall, 'max_force': max_force}
    dico.update(split_dict)
    dico.update(main_dict)
    dico.update(backup_dict)

    data = pd.DataFrame(data=dico)

    test_results = folder + 'TEST.csv'
    test_description = folder + 'description.txt'

    description = augustin.rig.describe()
    file = open(test_description, 'w')
    for items in description:
        file.write(items + '\n')

    file.close()
    data.to_csv(test_results)
    return data


def run_and_play(length, height, setup, maxt, leg_length, weight, leash_length, power, position, dt, steps, max_steps,
                 folder, settle_time):
    # Create the spot
    scale = 1280 / length  # this is for graphic display
    spot = Spot(length, 720 / scale - height)

    # Create the line with rig and spot
    line = Rig(setup, spot, maxt)

    print("Loading, please wait ...")

    # Create the slackliner
    augustin = Slackliner(leg_length, weight, leash_length, power, line, position[0], position[1])
    augustin.runs(settle_time, int(settle_time * steps / dt))  # let the line settle

    return line.num_segment, play(augustin, line, scale, max_steps, dt, steps, folder)


def load_and_play(saved_file, dt, steps, max_steps, folder):

    # Load the slackliner:
    in_file = open(saved_file, mode='r+b')

    augustin = pickle.load(in_file)

    in_file.close()

    # Recover the line
    line = augustin.rig

    # Calculate graphic scale
    scale = 1280 / line.spot.g  # this is for graphic display

    return line.num_segment, play(augustin, line, scale, max_steps, dt, steps, folder)
