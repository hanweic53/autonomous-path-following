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
    def __init__(self, x=0.0, y=0.0, heading_rad=0.0, 
                 longitudinal_velocity_mps=0.0, acceleration_mps2=0.0, front_wheel_angle_rad=0.0):
        self.time_from_start = 0.0 # seconds
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
            self.time_from_start, self.x, self.y, self.heading_rad, self.longitudinal_velocity_mps)

starting_point = TrajectoryPoint(longitudinal_velocity_mps=3.0) 
length = 100.0
discretization_m = 1.0
speed_max = 35.0
speed_increments = 0.33
stopping_decl = 3.0
heading_rate = 0.0
heading_rate_max = 1.0
heading_rate_increments = 0.0001

def create_curved_trajectory(
        init_point, 
        length, discretization_m,
        speed_max, speed_increments, stopping_decel,
        heading_rate, heading_rate_max, heading_rate_increments
    ):
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

        for i in range(1, num_points - 1):
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

def create_annotation():
    annotation = ax.annotate("", 
        xy=(0,0), xytext=(-10, 30),
        textcoords="offset points",
        bbox=dict(boxstyle="round", fc="w"),
        arrowprops=dict(arrowstyle="->"))
    annotation.set_visible(False)
    return annotation

annotation = create_annotation()

def plot_trajectory(trajectory):
    x = []
    y = []
    vel = []
    time = []
    
    for i in range(0, len(trajectory.points)):
        point = trajectory.points[i]
        x.append(point.x)
        y.append(point.y)
        vel.append(point.longitudinal_velocity_mps)
        time.append(point.time_from_start)
    
    x = np.array(x)
    y = np.array(y)
    vel = np.array(vel)
    time = np.array(time)

    ax.set_ylim(-60,60)
    fig.set_size_inches(14, 9)
    sc = plt.scatter(x=x, y=y, c=vel, cmap="copper_r", vmin=0, vmax=50)
    cbar = plt.colorbar()
    cbar.set_label("Vel m/s", rotation=360)
    
    ax.set_xlabel('Longitudinal Position X/m')
    ax.set_ylabel("Lateral Position Y/m")
    ax.set_title('Planned Trajectory')

    def update_annotation(ind):
        index = ind["ind"][0]
        pos = sc.get_offsets()[index]
        annotation.xy = pos
        text = "{}. \n{:.3f}s".format(index + 1, time[index])
        annotation.set_text(text)
        annotation.get_bbox_patch().set_facecolor("cyan")
        annotation.get_bbox_patch().set_alpha(0.4)
    
    def hover(event):
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

    # Make a horizontal slider to control the frequency.
    axheading = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)
    heading_slider = Slider(
        ax=axheading,
        label='Heading rate increment',
        valmin=0.0001,
        valmax=0.001,
        valinit= heading_rate_increments
    )

    # The function to be called anytime a slider's value changes
    def update(val):
        new_trajectory = create_curved_trajectory(
            init_point=starting_point, length=length,
            discretization_m=discretization_m,
            speed_max=speed_max, speed_increments=speed_increments,
            stopping_decel=stopping_decl, heading_rate=heading_rate,
            heading_rate_max=heading_rate_max, 
            heading_rate_increments=heading_slider.val)

        xy = []
        for i in range(0, len(new_trajectory.points)):
            point = new_trajectory.points[i]
            xy.append((point.x, point.y))
       
        sc.set_offsets(xy)
        fig.canvas.draw_idle()
    
    heading_slider.on_changed(update)

    fig.canvas.mpl_connect("motion_notify_event", hover)
    # adjust the main plot to make room for the sliders
    plt.subplots_adjust(bottom=0.25)
    plt.show()

def main(args=None):
    curved_trajectory= create_curved_trajectory(init_point=starting_point, length=length, 
                                                discretization_m=discretization_m,
                                                speed_max=speed_max, speed_increments=speed_increments, 
                                                stopping_decel=stopping_decl, heading_rate=heading_rate, 
                                                heading_rate_max=heading_rate_max, 
                                                heading_rate_increments=heading_rate_increments)
    
    # print(curved_trajectory)
    plot_trajectory(curved_trajectory)

if __name__ == '__main__':
    main()