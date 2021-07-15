from tkinter.constants import N

import math

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

import numpy as np
from numpy.lib.arraypad import pad

import csv

def to_mps(kmph):
    return (kmph * 1000)/3600

def to_kmph(mps):
    return (mps * 3600) / 1000

class Trajectory:
    capacity = 100 # max. length in Trajectory.msg

    def __init__(self):
        self.points = []

    def __str__(self):
        length = len(self.points)
        
        for i in range(0, length):
            print(str(i) + ". ", end = '')
            print(self.points[i])
        
        print(str(length) + " points")
        ret = ''
        return ret

class TrajectoryPoint:
    def __init__(self, time_from_start=0.0, x=0.0, y=0.0, heading_rad=0.0, 
                 longitudinal_velocity_mps=0.0, acceleration_mps2=0.0):
        self.time_from_start = time_from_start # seconds
        self.x = x
        self.y = y
        self.heading_rad = heading_rad
        self.longitudinal_velocity_mps = longitudinal_velocity_mps
        # self.lateral_velocity_mps = 0.0
        self.acceleration_mps2 = acceleration_mps2
        # self.heading_rate_rps = 0.0
        # self.front_wheel_angle_rad = front_wheel_angle_rad
        # self.rear_wheel_angle_rad = 0.0

    def __str__(self):
        return "{:.3f}s ({:.3f}, {:.3f}), heading = {:.6f}rad, velocity = {:.2f}m/s {:.2f}km/h".format(
            self.time_from_start, self.x, self.y, self.heading_rad, 
            self.longitudinal_velocity_mps, to_kmph(self.longitudinal_velocity_mps))

# params for sliders generation
initial_speed = None
initial_speed_valmin = None
initial_speed_valmax = None
final_speed = None
final_speed_valmin = None
final_speed_valmax = None
heading_rate_increments = None
heading_rate_increments_valmin = None
heading_rate_increments_valmax = None

def create_lane_change_trajectory():
    f = open('trajectories/lane_change_trajectory.csv', 'w')
    writer = csv.writer(f)
    writer.writerow(['time_from_start', "x", 'y', "heading_degrees", "longitudinal_velocity_mps",
        'acceleration_mps2'])
    
    # set params for plot
    ax.set_ylim(-20, 20)
    ax.set_xlim(-10, 120)

    # set params for trajectory    
    length = 100.0
    discretization_m = 1.0
    speed_increments = 0.15
    speed_max = 35.0
    stopping_decel = 1.0
    heading_rate = 0.0
    heading_rate_max = 1.0

    global heading_rate_increments, heading_rate_increments_valmin, heading_rate_increments_valmax
    if heading_rate_increments_valmin == None:
        heading_rate_increments = 0.00018 # slider
        heading_rate_increments_valmin = 0.0001
        heading_rate_increments_valmax = 0.001
    global initial_speed, initial_speed_valmin, initial_speed_valmax
    if initial_speed == None:
        initial_speed = 3.0 # slider
        initial_speed_valmin = 0.0
        initial_speed_valmax = 20.0
    global final_speed, final_speed_valmin, final_speed_valmax
    if final_speed == None:
        final_speed = 3.0 # slider
        final_speed_valmin = 0.0
        final_speed_valmax = 20.0

    trajectory_msg = Trajectory()
    
    num_points_max = Trajectory.capacity
    num_points = int(length / discretization_m)  
    if num_points > num_points_max:
        num_points = num_points_max
        print("Only 100 points available - discretization set to %s"
            % float(length / num_points_max)
        )
    discretization_distance_m = float(length / num_points)

    # start at base_link
    first_point = TrajectoryPoint(longitudinal_velocity_mps=initial_speed)
    trajectory_msg.points.append(first_point)
    # write into csv
    writer.writerow([first_point.time_from_start, first_point.x, first_point.y, first_point.heading_rad, first_point.longitudinal_velocity_mps, first_point.acceleration_mps2])

    decelerating = False
    speed = first_point.longitudinal_velocity_mps
    seconds = 0.0
    if speed > 0:
        seconds = float(discretization_distance_m / speed)
    cur_x = first_point.x
    cur_y = first_point.y
    heading_angle = math.degrees(first_point.heading_rad)
    prev_heading_angle = heading_angle
    prev_speed = speed

    for i in range(2, num_points + 1):
        # update speed profile
        if not decelerating:
            speed += speed_increments
            
            next_speed = speed + speed_increments
            predicted_stopping_time = (next_speed - final_speed) / stopping_decel
            predicted_stopping_distance = next_speed * predicted_stopping_time \
                - 0.5 * stopping_decel * predicted_stopping_time * predicted_stopping_time
            if ((num_points - i - 1) * discretization_distance_m) <= predicted_stopping_distance:
                decelerating = True

        speed = min(speed, speed_max)

        if speed > 0:
            seconds_delta = float(discretization_distance_m / speed)
            seconds += seconds_delta
            if decelerating:
                speed -= stopping_decel * seconds_delta
                speed = max(final_speed, speed)

        # update heading
        if i >= round(num_points * 0.2) and i < round(num_points * 0.6):
            heading_angle -= heading_rate
            heading_rate += heading_rate_increments
            heading_rate = max(-heading_rate_max, min(heading_rate_max, heading_rate))
        elif i >= round(num_points * 0.6) and i < round(num_points * 0.8):
            heading_angle += heading_rate
            heading_rate += heading_rate_increments
            heading_rate = max(-heading_rate_max, min(heading_rate_max, heading_rate))
            if heading_angle >= 0:
                heading_angle = 0

        # fillup trajectory point
        trajectory_point = TrajectoryPoint()
        trajectory_point.time_from_start = seconds

        cur_x += discretization_m * np.cos(heading_angle)
        cur_y += discretization_m * np.sin(heading_angle)
        trajectory_point.x = cur_x
        trajectory_point.y = cur_y
        
        trajectory_point.heading_rad = math.radians(heading_angle)
        trajectory_point.longitudinal_velocity_mps = float(speed)
        trajectory_point.acceleration_mps2 = (speed - prev_speed) / seconds
        trajectory_point.heading_rate_rps = (heading_angle - prev_heading_angle) / seconds

        trajectory_msg.points.append(trajectory_point)
        # write into csv
        # writing heading angle in degrees because Autoware's Heading requires conversion into Complex32 from degrees
        writer.writerow([trajectory_point.time_from_start, trajectory_point.x, trajectory_point.y, heading_angle, trajectory_point.longitudinal_velocity_mps, trajectory_point.acceleration_mps2])

        prev_heading_angle = heading_angle
        prev_speed = speed

    f.close()
    return trajectory_msg     

# higher curvature, lower speed than highway bend
def create_junction_turning_trajectory():
    # set params for plot
    ax.set_ylim(-70, 5)
    ax.set_xlim(-10, 120)

    # set params for trajectory    
    length = 100.0
    discretization_m = 1.0
    speed_increments = 0.15
    speed_max = 35.0
    stopping_decel = 1.0
    heading_rate = 0.0
    heading_rate_max = 1.0

    global heading_rate_increments, heading_rate_increments_valmin, heading_rate_increments_valmax
    if heading_rate_increments_valmin == None:
        heading_rate_increments = 0.005 # slider
        heading_rate_increments_valmin = 0.001
        heading_rate_increments_valmax = 0.005
    global initial_speed, initial_speed_valmin, initial_speed_valmax
    if initial_speed == None:
        initial_speed = 3.0 # slider
        initial_speed_valmin = 0.0
        initial_speed_valmax = 20.0
    global final_speed, final_speed_valmin, final_speed_valmax
    if final_speed == None:
        final_speed = 3.0 # slider
        final_speed_valmin = 0.0
        final_speed_valmax = 20.0

    trajectory_msg = Trajectory()
    
    num_points_max = Trajectory.capacity
    num_points = int(length / discretization_m)  
    if num_points > num_points_max:
        num_points = num_points_max
        print("Only 100 points available - discretization set to %s"
            % float(length / num_points_max)
        )
    discretization_distance_m = float(length / num_points)

    # start at base_link
    first_point = TrajectoryPoint(longitudinal_velocity_mps=initial_speed)
    trajectory_msg.points.append(first_point)

    decelerating = False
    speed = first_point.longitudinal_velocity_mps
    seconds = 0.0
    if speed > 0:
        seconds = float(discretization_distance_m / speed)
    cur_x = first_point.x
    cur_y = first_point.y
    heading_angle = math.degrees(first_point.heading_rad)
    prev_heading_angle = heading_angle
    prev_speed = speed

    for i in range(2, num_points + 1):
        # update speed profile
        if not decelerating:
            speed += speed_increments
            
            next_speed = speed + speed_increments
            predicted_stopping_time = (next_speed - final_speed) / stopping_decel
            predicted_stopping_distance = next_speed * predicted_stopping_time \
                - 0.5 * stopping_decel * predicted_stopping_time * predicted_stopping_time
            if ((num_points - i - 1) * discretization_distance_m) <= predicted_stopping_distance:
                decelerating = True

        speed = min(speed, speed_max)

        if speed > 0:
            seconds_delta = float(discretization_distance_m / speed)
            seconds += seconds_delta
            if decelerating:
                speed -= stopping_decel * seconds_delta
                speed = max(final_speed, speed)

        # update heading
        if i >= round(num_points * 0.2) and i < round(num_points * 0.8):
            heading_angle -= heading_rate
            heading_rate += heading_rate_increments
            heading_rate = max(-heading_rate_max, min(heading_rate_max, heading_rate))
            if heading_angle < -1.5708:
                heading_angle = -1.5708
            

        # fillup trajectory point
        trajectory_point = TrajectoryPoint()
        trajectory_point.time_from_start = seconds

        cur_x += discretization_m * np.cos(heading_angle)
        cur_y += discretization_m * np.sin(heading_angle)
        trajectory_point.x = cur_x
        trajectory_point.y = cur_y
        
        trajectory_point.heading_rad = math.radians(heading_angle)
        trajectory_point.longitudinal_velocity_mps = float(speed)
        trajectory_point.acceleration_mps2 = (speed - prev_speed) / seconds
        trajectory_point.heading_rate_rps = (heading_angle - prev_heading_angle) / seconds

        trajectory_msg.points.append(trajectory_point)
        prev_heading_angle = heading_angle
        prev_speed = speed

    return trajectory_msg

# lower curvature, higher speed than junction turning
def create_highway_bend_trajectory():
    # set params for plot
    ax.set_ylim(-70, 5)
    ax.set_xlim(-10, 120)

    # set params for trajectory    
    length = 100.0
    discretization_m = 1.0
    speed_increments = 0.15
    speed_max = 35.0
    stopping_decel = 1.0
    heading_rate = 0.0
    heading_rate_max = 1.0

    global heading_rate_increments, heading_rate_increments_valmin, heading_rate_increments_valmax
    if heading_rate_increments_valmin == None:
        heading_rate_increments = 0.001 # slider
        heading_rate_increments_valmin = 0.0005
        heading_rate_increments_valmax = 0.003
    global initial_speed, initial_speed_valmin, initial_speed_valmax
    if initial_speed == None:
        initial_speed = 14.0 # slider
        initial_speed_valmin = 0.0
        initial_speed_valmax = 20.0
    global final_speed, final_speed_valmin, final_speed_valmax
    if final_speed == None:
        final_speed = 14.0 # slider
        final_speed_valmin = 0.0
        final_speed_valmax = 20.0

    trajectory_msg = Trajectory()
    
    num_points_max = Trajectory.capacity
    num_points = int(length / discretization_m)  
    if num_points > num_points_max:
        num_points = num_points_max
        print("Only 100 points available - discretization set to %s"
            % float(length / num_points_max)
        )
    discretization_distance_m = float(length / num_points)

    # start at base_link
    first_point = TrajectoryPoint(longitudinal_velocity_mps=initial_speed)
    trajectory_msg.points.append(first_point)

    decelerating = False
    speed = first_point.longitudinal_velocity_mps
    seconds = 0.0
    if speed > 0:
        seconds = float(discretization_distance_m / speed)
    cur_x = first_point.x
    cur_y = first_point.y
    heading_angle = math.degrees(first_point.heading_rad)
    prev_heading_angle = heading_angle
    prev_speed = speed

    for i in range(2, num_points + 1):
        # update speed profile
        if not decelerating:
            speed += speed_increments
            
            next_speed = speed + speed_increments
            predicted_stopping_time = (next_speed - final_speed) / stopping_decel
            predicted_stopping_distance = next_speed * predicted_stopping_time \
                - 0.5 * stopping_decel * predicted_stopping_time * predicted_stopping_time
            if ((num_points - i - 1) * discretization_distance_m) <= predicted_stopping_distance:
                decelerating = True

        speed = min(speed, speed_max)

        if speed > 0:
            seconds_delta = float(discretization_distance_m / speed)
            seconds += seconds_delta
            if decelerating:
                speed -= stopping_decel * seconds_delta
                speed = max(final_speed, speed)

        # update heading
        if i >= round(num_points * 0.2) and i < round(num_points * 0.8):
            heading_angle -= heading_rate
            heading_rate += heading_rate_increments
            heading_rate = max(-heading_rate_max, min(heading_rate_max, heading_rate))
            if heading_angle < -1.5708:
                heading_angle = -1.5708
            

        # fillup trajectory point
        trajectory_point = TrajectoryPoint()
        trajectory_point.time_from_start = seconds

        cur_x += discretization_m * np.cos(heading_angle)
        cur_y += discretization_m * np.sin(heading_angle)
        trajectory_point.x = cur_x
        trajectory_point.y = cur_y
        
        trajectory_point.heading_rad = math.radians(heading_angle)
        trajectory_point.longitudinal_velocity_mps = float(speed)
        trajectory_point.acceleration_mps2 = (speed - prev_speed) / seconds
        trajectory_point.heading_rate_rps = (heading_angle - prev_heading_angle) / seconds

        trajectory_msg.points.append(trajectory_point)
        prev_heading_angle = heading_angle
        prev_speed = speed

    return trajectory_msg

fig, ax = plt.subplots()

# annotation template to display index, time and velocity of waypoints 
def create_annotation():
    annotation = ax.annotate("", 
        xy=(0,0), xytext=(-10, 30),
        textcoords="offset points",
        bbox=dict(boxstyle="round", fc="w"),
        arrowprops=dict(arrowstyle="->"))
    annotation.set_visible(False)
    return annotation
annotation = create_annotation()

x = []
y = []
vel = []
vel_kmph = []
time = []
heading = []

def plot_trajectory(trajectory):
    global x, y, vel, vel_kmph, time, heading
    
    for i in range(0, len(trajectory.points)):
        point = trajectory.points[i]
        x.append(point.x)
        y.append(point.y)
        vel.append(point.longitudinal_velocity_mps)
        vel_kmph.append(to_kmph(point.longitudinal_velocity_mps))
        time.append(point.time_from_start)
        heading.append(point.heading_rad)
    
    x = np.array(x)
    y = np.array(y)
    vel = np.array(vel)
    vel_kmph = np.array(vel_kmph)
    time = np.array(time)
    heading = np.array(heading)

    fig.set_size_inches(18, 14)

    # to simulate the vehicle and display the headings of the waypoints
    def apply_waypoint_heading(idx):
        vehicle_length = 6
        endx = x[idx] + vehicle_length * math.cos(math.degrees(heading[idx]))
        endy = y[idx] + vehicle_length * math.sin(math.degrees(heading[idx]))
        return (endx, endy)

    init_endx, init_endy = apply_waypoint_heading(0)
    line = ax.plot([x[0], init_endx], [y[0], init_endy])[0]
    sc = plt.scatter(x=x, y=y, c=vel_kmph, cmap="copper_r", vmin=0, vmax=100)
    cbar = plt.colorbar(pad=0.02)
    cbar.set_label("Vel km/h", rotation=360, labelpad=30)
    
    ax.set_xlabel('Longitudinal Position X/m')
    ax.set_ylabel("Lateral Position Y/m")
    ax.set_title('Planned Trajectory')
    
    def hover(event):
        def update_annotation(ind):
            index = ind["ind"][0]
            pos = sc.get_offsets()[index]
            annotation.xy = pos
            text = "{}. \n({:.3f},{:.3f})\nTimestamp: {:.3f}s \nVelocity: {:.2f}m/s {:.2f}km/h \nHeading: {:.3f}\N{DEGREE SIGN} {:.3f}rad".format(index, x[index], y[index], time[index], vel[index], to_kmph(vel[index]), math.degrees(heading[index]), heading[index])
            annotation.set_text(text)
            annotation.get_bbox_patch().set_facecolor("cyan")
            annotation.get_bbox_patch().set_alpha(0.4)
        
        is_visible = annotation.get_visible()
        if event.inaxes == ax:
            cont, ind = sc.contains(event)
            if cont:
                update_annotation(ind)
                annotation.set_visible(True)
                fig.canvas.draw_idle()
            else:
                if is_visible:
                    annotation.set_visible(False)
                    fig.canvas.draw_idle()
    
    axcolor = 'lightgoldenrodyellow'

    # Make a vertical slider to control the initial speed.
    axinitialspeed = plt.axes([0.02, 0.25, 0.0225, 0.63], facecolor=axcolor)
    initial_speed_slider = Slider(
        ax=axinitialspeed,
        label='Starting speed\n(km/h)',
        valmin=to_kmph(initial_speed_valmin),
        valmax=to_kmph(initial_speed_valmax),
        valinit=to_kmph(initial_speed),
        valfmt = '%0.3f',
        orientation= "vertical"
    )

    # Make a vertical slider to control the final speed.
    axfinalspeed = plt.axes([0.085, 0.25, 0.0225, 0.63], facecolor=axcolor)
    final_speed_slider = Slider(
        ax=axfinalspeed,
        label='Final speed\n(km/h)',
        valmin=to_kmph(final_speed_valmin),
        valmax=to_kmph(final_speed_valmax),
        valinit=to_kmph(final_speed),
        valfmt = '%0.3f',
        orientation= "vertical"
    )

    # Make a vertical slider to control the heading rate increments.
    axheading = plt.axes([0.16, 0.25, 0.0225, 0.63], facecolor=axcolor)
    heading_slider = Slider(
        ax=axheading,
        label='Heading Increments\n(rad/s)',
        valmin=heading_rate_increments_valmin,
        valmax=heading_rate_increments_valmax,
        valinit= heading_rate_increments,
        valfmt = '%0.5f',
        orientation= "vertical"
    )

    # Make a horizontal slider to control the index of waypoints.
    axwaypoints = plt.axes([0.25, 0.1, 0.63, 0.03], facecolor=axcolor)
    index_slider = Slider(
        ax=axwaypoints,
        label='Index',
        valmin=0,
        valmax=len(x) - 1,
        valinit= 0,
        valfmt = '%0.0f',
        orientation= "horizontal")
    
    # Create a button to reset the sliders to initial values.
    resetax = plt.axes([0.1, 0.15, 0.1, 0.04])
    button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')
    def reset(event):
        index_slider.reset()
        initial_speed_slider.reset()
        final_speed_slider.reset()
        heading_slider.reset()

    button.on_clicked(reset)

    # function to be called when the index slider moves
    def update_index_plot(val):
        idx = round(index_slider.val)
        endx, endy = apply_waypoint_heading(idx)
        line.set_xdata([x[idx], endx])
        line.set_ydata([y[idx], endy])
    
    # function to be called when the trajectory params' sliders move
    def update_trajectory_plot(val):
        global initial_speed, final_speed, heading_rate_increments, x, y, vel, vel_kmph, time, heading
        initial_speed = to_mps(initial_speed_slider.val)
        final_speed = to_mps(final_speed_slider.val)
        heading_rate_increments = heading_slider.val

        new_trajectory = get_trajectory()
        # print(new_trajectory)
        new_trajectory_len = len(new_trajectory.points)
        xy = []
        for i in range(0, new_trajectory_len):
            point = new_trajectory.points[i]
            
            # mutate the arrays when the trajectory was initially plotted
            x[i] = point.x
            y[i] = point.y
            vel[i] = point.longitudinal_velocity_mps
            vel_kmph[i] = to_kmph(point.longitudinal_velocity_mps)
            time[i] = point.time_from_start
            heading[i] = point.heading_rad
            
            xy.append((point.x, point.y))
        
        # some points from the original trajectory will not be overwritten
        if new_trajectory_len < len(trajectory.points):
            x = x[:new_trajectory_len]
            y = y[:new_trajectory_len]
            vel = vel[:new_trajectory_len]
            vel_kmph = vel_kmph[:new_trajectory_len]
            time = time[:new_trajectory_len]
            heading = heading[:new_trajectory_len]

        # Update length of slider when new trajectory has different # points
        index_slider.set_valmax = len(x) - 1

        # when the trajectory changes, the index plot will always change too
        update_index_plot(index_slider.val)
        sc.set_offsets(xy)
        sc.set_cmap("copper_r")
        fig.canvas.draw_idle()
    
    initial_speed_slider.on_changed(update_trajectory_plot)
    final_speed_slider.on_changed(update_trajectory_plot)
    heading_slider.on_changed(update_trajectory_plot)
    index_slider.on_changed(update_index_plot)

    fig.canvas.mpl_connect("motion_notify_event", hover)
    # adjust the main plot to make room for the sliders
    plt.subplots_adjust(left=0.25, bottom=0.25, right=1.04)
    plt.show()

def get_trajectory():
    return create_lane_change_trajectory()
    # return create_junction_turning_trajectory()
    # return create_highway_bend_trajectory()

def main(args=None):
    # print(get_trajectory())
    plot_trajectory(get_trajectory())

if __name__ == '__main__':
    main()