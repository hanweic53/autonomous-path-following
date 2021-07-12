import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import numpy as np
import math

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
        return "{:.3f}s ({:.3f}, {:.3f}), heading = {:.6f}, velocity = {:.2f}".format(
            self.time_from_start, self.x, self.y, self.heading_rad, 
            self.longitudinal_velocity_mps)

# common params for trajectory slider generation
initial_speed = None
initial_speed_valmin = None
initial_speed_valmax = None
heading_rate_increments = None
heading_rate_increments_valmin = None
heading_rate_increments_valmax = None

def create_lane_change_trajectory():
    # set params for plot
    ax.set_ylim(-20, 20)
    ax.set_xlim(-10, 120)

    # set params for trajectory    
    length = 100.0
    discretization_m = 1.0
    speed_increments = 0.33
    speed_max = 35.0
    stopping_decel = 3.0
    heading_rate = 0.0
    heading_rate_max = 1.0

    global heading_rate_increments, heading_rate_increments_valmin, heading_rate_increments_valmax
    if heading_rate_increments_valmin == None:
        heading_rate_increments = 0.00015 # slider
        heading_rate_increments_valmin = 0.0001
        heading_rate_increments_valmax = 0.001
    global initial_speed, initial_speed_valmin, initial_speed_valmax
    if initial_speed == None:
        initial_speed = 3.0 # slider
        initial_speed_valmin = 0.0
        initial_speed_valmax = 10.0
    
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

    stopping = False
    speed = first_point.longitudinal_velocity_mps
    seconds = 0.0
    if speed > 0:
        seconds = float(discretization_distance_m / speed)
    cur_x = first_point.x
    cur_y = first_point.y
    heading_angle = math.degrees(first_point.heading_rad)
    prev_heading_angle = heading_angle
    prev_speed = speed

    for i in range(1, round(num_points * 0.2)):
        # update speed profile
        speed += speed_increments
        speed = min(speed, speed_max)


        if speed > 0:
            seconds_delta = float(discretization_distance_m / speed)
            seconds += seconds_delta

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

    for i in range(round(num_points * 0.2), round(num_points * 0.6)):
        # update speed profile
        if not stopping:
            speed += speed_increments
            stopping_time = speed / stopping_decel
            stopping_distance = \
                speed * stopping_time \
                - 0.5 * stopping_decel * stopping_time * stopping_time
            if ((num_points - i) * discretization_distance_m) <= stopping_distance:
                stopping = True

        speed = min(speed, speed_max)

        if i == (num_points - 2):
            speed = 0.0

        if speed > 0:
            seconds_delta = float(discretization_distance_m / speed)
            seconds += seconds_delta
            if stopping:
                speed -= stopping_decel * seconds_delta
                speed = max(0.0, speed)

        # update heading
        heading_angle -= heading_rate
        heading_rate += heading_rate_increments
        heading_rate = max(-heading_rate_max, min(heading_rate_max, heading_rate))

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
    
    for i in range(round(num_points * 0.6), round(num_points * 0.8)):
        # update speed profile
        if not stopping:
            speed += speed_increments
            stopping_time = speed / stopping_decel
            stopping_distance = \
                speed * stopping_time \
                - 0.5 * stopping_decel * stopping_time * stopping_time
            if ((num_points - i) * discretization_distance_m) <= stopping_distance:
                stopping = True

        speed = min(speed, speed_max)

        if i == (num_points - 2):
            speed = 0.0

        if speed > 0:
            seconds_delta = float(discretization_distance_m / speed)
            seconds += seconds_delta
            if stopping:
                speed -= stopping_decel * seconds_delta
                speed = max(0.0, speed)

        # update heading
        heading_angle += heading_rate
        heading_rate += heading_rate_increments
        if (heading_angle > 0):
            heading_angle = 0
        heading_rate = max(-heading_rate_max, min(heading_rate_max, heading_rate))

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

    for i in range(round(num_points * 0.8), num_points):
        # update speed profile
        if not stopping:
            speed += speed_increments
            stopping_time = speed / stopping_decel
            stopping_distance = \
                speed * stopping_time \
                - 0.5 * stopping_decel * stopping_time * stopping_time
            if ((num_points - i) * discretization_distance_m) <= stopping_distance:
                stopping = True

        speed = min(speed, speed_max)

        if i == (num_points - 2):
            speed = 0.0

        if speed > 0:
            seconds_delta = float(discretization_distance_m / speed)
            seconds += seconds_delta
            if stopping:
                speed -= stopping_decel * seconds_delta
                speed = max(0.0, speed)

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

def create_curved_trajectory():
    ax.set_ylim(-60, 60)
    ax.set_xlim(-10, 120)
    init_point = TrajectoryPoint(longitudinal_velocity_mps=3.0)
    length = 100.0
    discretization_m = 1.0
    speed_max = 35.0
    stopping_decel = 3.0
    heading_rate = 0.0
    heading_rate_max = 1.0

    global heading_rate_increments, heading_rate_increments_valmin, heading_rate_increments_valmax
    if heading_rate_increments_valmin == None:
        heading_rate_increments = 0.0001 # slider
        heading_rate_increments_valmin = 0.0001
        heading_rate_increments_valmax = 0.001
    global speed_increments, speed_increments_valmin, speed_increments_valmax
    if speed_increments == None:
        speed_increments = 0.33 # slider
        speed_increments_valmin = 0.33
        speed_increments_valmax = 1.0
    
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
    trajectory_msg.points.append(init_point)

    stopping = False
    speed = init_point.longitudinal_velocity_mps
    seconds = float(discretization_distance_m / speed)

    cur_x = init_point.x
    cur_y = init_point.y
    heading_angle = math.degrees(init_point.heading_rad)
    prev_heading_angle = heading_angle
    prev_speed = speed

    for i in range(1, num_points):
        # update speed profile
        if not stopping:
            speed += speed_increments
            stopping_time = speed / stopping_decel
            stopping_distance = \
                speed * stopping_time \
                - 0.5 * stopping_decel * stopping_time * stopping_time
            if ((num_points - i) * discretization_distance_m) <= stopping_distance:
                stopping = True

        speed = min(speed, speed_max)

        if i == (num_points - 2):
            speed = 0.0

        if speed > 0:
            seconds_delta = float(discretization_distance_m / speed)
            seconds += seconds_delta
            if stopping:
                speed -= stopping_decel * seconds_delta
                speed = max(0.0, speed)

        # update heading
        heading_angle += heading_rate
        heading_rate += heading_rate_increments
        heading_rate = max(-heading_rate_max, min(heading_rate_max, heading_rate))

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
time = []
heading = []

def plot_trajectory(trajectory):
    global x, y, vel, time, heading
    
    for i in range(0, len(trajectory.points)):
        point = trajectory.points[i]
        x.append(point.x)
        y.append(point.y)
        vel.append(point.longitudinal_velocity_mps)
        time.append(point.time_from_start)
        heading.append(point.heading_rad)
    
    x = np.array(x)
    y = np.array(y)
    vel = np.array(vel)
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
    sc = plt.scatter(x=x, y=y, c=vel, cmap="copper_r", vmin=0, vmax=50)
    cbar = plt.colorbar()
    cbar.set_label("Vel m/s", rotation=360)
    
    ax.set_xlabel('Longitudinal Position X/m')
    ax.set_ylabel("Lateral Position Y/m")
    ax.set_title('Planned Trajectory')
    
    def hover(event):
        def update_annotation(ind):
            index = ind["ind"][0]
            pos = sc.get_offsets()[index]
            annotation.xy = pos
            text = "{}. \n({:.3f},{:.3f})\nTimestamp: {:.3f}s \nVelocity: {:.2f}m/s\
                \nHeading: {:.3f}\N{DEGREE SIGN}".format(index, x[index], y[index], time[index], vel[index], math.degrees(heading[index]))
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
        label='Starting speed\n(m/s)',
        valmin=initial_speed_valmin,
        valmax=initial_speed_valmax,
        valinit=initial_speed,
        valfmt = '%0.3f',
        orientation= "vertical"
    )

    # Make a vertical slider to control the initial speed.
    axendingspeed = plt.axes([0.085, 0.25, 0.0225, 0.63], facecolor=axcolor)
    ending_speed_slider = Slider(
        ax=axendingspeed,
        label='Final speed\n(m/s)',
        valmin=initial_speed_valmin,
        valmax=initial_speed_valmax,
        valinit=initial_speed,
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

    # function to be called when the index slider moves
    def update_index_plot(val):
        idx = round(index_slider.val)
        endx, endy = apply_waypoint_heading(idx)
        line.set_xdata([x[idx], endx])
        line.set_ydata([y[idx], endy])
    
    # function to be called when the trajectory params' sliders move
    def update_trajectory_plot(val):
        global initial_speed, heading_rate_increments, x, y, vel, time, heading
        initial_speed = initial_speed_slider.val
        heading_rate_increments = heading_slider.val

        new_trajectory = get_trajectory()
        new_trajectory_len = len(new_trajectory.points)
        xy = []
        for i in range(0, new_trajectory_len):
            point = new_trajectory.points[i]
            
            # mutate the arrays when the trajectory was initially plotted
            x[i] = point.x
            y[i] = point.y
            vel[i] = point.longitudinal_velocity_mps
            time[i] = point.time_from_start
            heading[i] = point.heading_rad
            
            xy.append((point.x, point.y))
        
        # some points from the original trajectory will not be overwritten
        if new_trajectory_len < len(trajectory.points):
            x = x[:new_trajectory_len]
            y = y[:new_trajectory_len]
            vel = y[:new_trajectory_len]
            time = y[:new_trajectory_len]
            heading = y[:new_trajectory_len]

        # Update length of slider when new trajectory has different # points
        index_slider.set_valmax = len(x) - 1

        # when the trajectory changes, the index plot will always change too
        update_index_plot(index_slider.val)
        sc.set_offsets(xy)
        sc.set_cmap("copper_r")
        fig.canvas.draw_idle()
    
    initial_speed_slider.on_changed(update_trajectory_plot)
    ending_speed_slider.on_changed(update_trajectory_plot)
    heading_slider.on_changed(update_trajectory_plot)
    index_slider.on_changed(update_index_plot)

    fig.canvas.mpl_connect("motion_notify_event", hover)
    # adjust the main plot to make room for the sliders
    plt.subplots_adjust(left=0.25, bottom=0.25, right=1.04)
    plt.show()

def get_trajectory():
    #return create_curved_trajectory()
    return create_lane_change_trajectory()

def main(args=None):
    # print(trajectory)
    plot_trajectory(get_trajectory())

if __name__ == '__main__':
    main()