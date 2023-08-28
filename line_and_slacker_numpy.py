from math import fabs, ceil
from random import randint
import pygame
import pandas as pd
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
        self.k = force / stretch
        self.d = d

    def describe(self):
        return f'lw:{self.lw * 1000} g/m_sc:{self.k / 1000:.2f}kN/m_dc:{self.d}'


class Segment(Element):
    def __init__(self, webbing, length, num_points, start_point, end_point, maxt=10000, mint=0):
        self.webbing = webbing
        self.len = length
        self.num_points = num_points  # number of points
        self.dl = length / (num_points + 1)
        self.dw = webbing.lw * self.dl
        self.k = webbing.k
        self.d = webbing.d
        self.maxt = maxt  # for color scale
        self.mint = mint
        self.points = np.linspace(start_point, end_point, num_points + 2)
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
        for i in range(self.num_points + 1):
            pygame.draw.line(win, (255, colors[i], colors[i]), (self.points[i, 0] * scale, self.points[i, 1] * scale),
                             (self.points[i + 1, 0] * scale, self.points[i + 1, 1] * scale), 3)


class CutEnd(Element):
    def __init__(self, segment, start, speed, w, d):
        self.w = w  # weight
        self.d = d  # drag
        self.segment = segment
        self.start = start  # bool (start vs end)
        if start:
            self.position = segment.points[0]
        else:
            self.position = segment.points[-1]
        self.acceleration = [0, 0]
        self.speed = speed

    def compute_acceleration(self):
        self.acceleration = [0, 9.81]
        if self.start:
            self.acceleration += self.segment.tensions_coordinates[0] / self.w
        else:
            self.acceleration -= self.segment.tensions_coordinates[-1] / self.w
        self.acceleration -= self.speed * np.fabs(self.speed) * self.d / self.w

    def compute_speed(self, dt):
        self.speed += self.acceleration * dt

    def compute_position(self, dt):
        self.position += self.speed * dt
        # Update position for end of connected segment
        if self.start:
            self.segment.points[0] = self.position
        else:
            self.segment.points[-1] = self.position


class Split(Element):
    def __init__(self, position, w, d, left_main, left_backup, right_main, right_backup):
        # position, weight, natural length, stretch constant for main and back-up, previous and next,
        # drag, max tension (for color scale)
        self.position = position
        self.speed = [0, 0]
        self.acceleration = [0, 0]
        self.w = w
        self.d = d
        self.left_main = left_main
        self.left_backup = left_backup
        self.right_main = right_main
        self.right_backup = right_backup

    def compute_acceleration(self):
        # forces that apply: elasticity and gravity
        self.acceleration = [0, 9.81]
        self.acceleration += self.right_backup.tensions_coordinates[0] / self.w
        self.acceleration += self.right_main.tensions_coordinates[0] / self.w
        self.acceleration -= self.left_backup.tensions_coordinates[-1] / self.w
        self.acceleration -= self.left_main.tensions_coordinates[-1] / self.w
        self.acceleration -= self.speed * np.fabs(self.speed) * self.d / self.w

    def compute_speed(self, dt):
        self.speed += self.acceleration * dt

    def compute_position(self, dt):
        self.position += self.speed * dt
        # Update positions for ends of connected segments
        self.right_backup.points[0] = self.position
        self.right_main.points[0] = self.position
        self.left_backup.points[-1] = self.position
        self.right_backup.points[-1] = self.position


class Spot:
    def __init__(self, g, h):
        # gap length, height
        self.g = g
        self.h = h
        self.anchor1 = Anchor(0, h)
        self.anchor2 = Anchor(g, h)

    def describe(self):
        return f'length: {self.g} m, height: {self.h} m'


class SetUpError(Exception):
    pass


class SetUp:
    def __init__(self, mains, backups, main_lengths, backup_lengths, points_per_meter):
        # Check that the parameters are correct
        if len(mains) != len(backups) or len(mains) != len(main_lengths) or len(mains) != len(backup_lengths):
            raise SetUpError
        if len(mains) == 0:
            raise SetUpError
        # Initialize
        self.mains = mains
        self.backups = backups
        self.main_lengths = main_lengths
        self.backup_lengths = backup_lengths
        self.num_segment = len(mains)
        self.total_length = sum(main_lengths)
        self.points_per_meter = points_per_meter


class Rig:
    def __init__(self, setup, spot, mint=0, maxt=10000):
        # setup: list of Segments not empty
        self.setup = setup
        self.spot = spot
        self.mint = mint
        self.maxt = maxt
        # calculate set-up mainline length and number of segments
        ratio = spot.g / self.setup.total_length  # for original placement of  the line
        self.mains = []  # list of Segments
        self.backups = []  # list of Segments
        self.splits = []  # list of Split
        self.tension_main = []  # update the tension after each time increment
        self.i_tension_main = []  # record the highest tension since last record
        self.tension_backup = []
        self.i_tension_backup = []

        # Compute the Segments
        start_x = 0
        end_x = 0
        for i in range(spot.num_segment):
            num_points = ceil(spot.main_lengths[i] * spot.points_per_meter)
            end_x += spot.main_lengths[i] * ratio
            self.mains.append(Segment(setup.mains[i], setup.main_lengths[i], num_points,
                                      [start_x, spot.h], [end_x, spot.h], maxt, mint))
            self.backups.append(Segment(setup.backups[i], setup.backups_lengths[i], num_points,
                                        [start_x, spot.h], [end_x, spot.h], maxt, mint))
            splitweight = setup.mains[i].lw * setup.main_lengths[i] / num_points \
                          + setup.backups[i].lw * setup.backups_lengths[i] / num_points
            splitdrag = setup.mains[i].d * setup.main_lengths[i] / num_points \
                        + setup.backups[i].d * setup.backups_lengths[i] / num_points
            if i != 0:
                splitweight_next = setup.mains[i].lw * setup.main_lengths[i] / num_points \
                                   + setup.backups[i].lw * setup.backups_lengths[i] / num_points
                splitdrag_next = setup.mains[i].d * setup.main_lengths[i] / num_points \
                                 + setup.backups[i].d * setup.backups_lengths[i] / num_points
                self.splits.append(
                    Split([start_x, spot.h], (splitweight + splitweight_next) / 2, (splitdrag + splitdrag_next) / 2,
                          self.mains[i - 1], self.backups[i - 1], self.mains[i], self.backups[i]))
            start_x += spot.main_lengths[i] * ratio

    def get_tension(self):
        tension_main = []
        for i, t in enumerate(self.i_tension_main):
            tension_main.append(t)
            self.i_tension_main[i] = 0
        tension_backup = []
        for i, t in enumerate(self.i_tension_backup):
            tension_backup.append(t)
            self.i_tension_backup[i] = 0
        return tension_main, tension_backup

    def draw(self, win, scale):
        for main in self.mains:
            main.draw(win, scale)
        for backup in self.backups:
            backup.draw(win, scale)

    def acceleration(self):
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

    def move(self, dt):
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
        dt = t / steps
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
        if self.unit < self.rig.segments[self.segment].np - 1:
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
            f = -(10000 / 0.25) * (self.y - self.leash_length - self.rig.mains[seg][unit].y) / self.leash_length
            self.leash_tension = max(fabs(f), self.leash_tension)
            self.rig.mains[seg][unit].ay -= f / self.rig.mains[seg][unit].w
        if self.y - self.leash_length > self.rig.backups[seg][unit].y:
            f = -(10000 / 0.25) * (self.y - self.leash_length - self.rig.backups[seg][unit].y) / self.leash_length
            self.leash_tension = max(fabs(f), self.leash_tension)
            self.rig.backups[seg][unit].ay -= f / self.rig.backups[seg][unit].w
        ay = (f + g) / self.weight - 0.1 * self.vy
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
        dt = t / steps
        for ii in range(steps):
            self.run(dt)

    def backup_fall(self, segment, unit=-1):
        if segment < self.rig.num_segment:
            self.backupfall = True
            self.backupfall_segment = segment
            if unit == -1:
                unit = randint(0, self.rig.segments[segment].np - 1)
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
            out_file = open(folder + 'save_state.pk', mode='w+b')

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
        leash_tension.append(augustin.get_tension() / 1000)
        height.append(-augustin.y)
        back_up_fall.append(augustin.backupfall_segment)
        split_tension, main_tension, backup_tension = augustin.rig.get_tension()
        for i in range(line.num_segment):
            if i < line.num_segment - 1:
                splits_tension_h[i].append(split_tension[i] / 1000)
            main_tension_h[i].append(main_tension[i] / 1000)
            backup_tension_h[i].append(backup_tension[i] / 1000)
        if line.num_segment > 1:
            max_force.append(max(max(main_tension), max(split_tension), max(backup_tension)) / 1000)
        else:
            max_force.append(max(max(main_tension), max(backup_tension)) / 1000)
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
