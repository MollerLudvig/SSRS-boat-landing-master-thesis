import matplotlib.pyplot as plt
from redis_communication import RedisClient
from time import sleep
import numpy as np

class Plotter:
    def __init__(self):
        self.rc = RedisClient()
        plt.ion()
        self.plots = {}

        self.drone_data = {}
        self.boat_data = {}

        # Initialize the trajectory plot
        fig_traj, ax_traj = plt.subplots()
        line1_traj, = ax_traj.plot([], [], 'bo-', label='Drone altitude')
        line2_traj, = ax_traj.plot([], [], 'go--', label='Z Wanted')
        ax_traj.set_xlabel("Distance since descent start (m)")
        ax_traj.set_ylabel("Altitude (m)")
        ax_traj.legend()

        self.plots["trajectory"] = {
            "fig": fig_traj,
            "ax": ax_traj,
            "line1": line1_traj,
            "line2": line2_traj,
            "hline": None
        }

        # Initialize sink rate plot
        fig_sr, ax_sr = plt.subplots()
        line1_sr, = ax_sr.plot([], [], 'go-', label='Wanted sink rate')
        line2_sr, = ax_sr.plot([], [], 'bo-', label='Needed sink rate')
        line3_sr, = ax_sr.plot([], [], 'ro-', label="Actual sink rate")
        ax_sr.set_xlabel("Distance since descent start (m)")
        ax_sr.set_ylabel("Sink rate (m/s)")
        ax_sr.legend()

        self.plots["sink_rate"] = {
            "fig": fig_sr,
            "ax": ax_sr,
            "line1": line1_sr,
            "line2": line2_sr,
            "line3": line3_sr
        }

        # Initialize altitude error plot
        fig_alterr, ax_alterr = plt.subplots()
        line1_alterr, = ax_alterr.plot([], [], 'go-', label='Altitude error')
        ax_alterr.set_xlabel("Distance since descent start (m)")
        ax_alterr.set_ylabel("Altitude error (m)")
        ax_alterr.legend()

        self.plots["altitude_error"] = {
            "fig": fig_alterr,
            "ax": ax_alterr,
            "line1": line1_alterr
        }

        # Initialize filter plot
        fig_filter, ax_filter = plt.subplots()
        line1_filter, = ax_filter.plot([], [], 'ro-', label='Filtered trajectory')
        line2_filter, = ax_filter.plot([], [], 'bo-', label='Raw trajectory')
        ax_filter.set_xlabel("Longitude")
        ax_filter.set_ylabel("Latitude")
        ax_filter.legend()

        self.plots["filter_xy"] = {
            "fig": fig_filter,
            "ax": ax_filter,
            "line1": line1_filter,
            "line2": line2_filter
        }

        # Initialize filter plot lat long
        fig_filter_latlon, ax_filter_latlon = plt.subplots()
        line1_filter_latlon, = ax_filter_latlon.plot([], [], 'ro-', label='Filtered trajectory')
        line2_filter_latlon, = ax_filter_latlon.plot([], [], 'bo-', label='Raw trajectory')
        ax_filter_latlon.set_xlabel("Latitude")
        ax_filter_latlon.set_ylabel("Longitude")
        ax_filter_latlon.legend()

        self.plots["filter_latlon"] = {
            "fig": fig_filter_latlon,
            "ax": ax_filter_latlon,
            "line1": line1_filter_latlon,
            "line2": line2_filter_latlon
        }
    
    def plot_trajectory(self):
        try:
            data = self.plots["trajectory"]
            ax = data["ax"]
            line1 = data["line1"]
            line2 = data["line2"]

            xs, altitudes, z_wanteds, Gr = self.drone_data["xs"], self.drone_data["altitude"], self.drone_data["z_wanted"], self.drone_data["Gr"]
            boat_altitude = self.boat_data["altitude"]

            if data["hline"]:
                data["hline"].remove()
            data["hline"] = ax.axhline(y=boat_altitude, color='red', linestyle='--', label='Boat altitude')

            line1.set_xdata(xs)
            line1.set_ydata(altitudes)
            line2.set_xdata(xs)
            line2.set_ydata(z_wanteds)

            ax.relim()
            ax.autoscale_view()
            ax.set_title(f"Glide ratio: 1/{int(1/Gr)}")

            plt.draw()
            plt.pause(0.01)
        except Exception as e:
            print("Waiting for stream:", e)

    def plot_sink_rate(self):
        try:
            data = self.plots["sink_rate"]
            ax = data["ax"]
            line1 = data["line1"]
            line2 = data["line2"]
            line3 = data["line3"]


            xs, wanted_sr, needed_sr, actual_sr, Gr = self.drone_data["xs"], self.drone_data["wanted_sr"], self.drone_data["needed_sr"], self.drone_data["actual_sr"], self.drone_data["Gr"]

            line1.set_xdata(xs)
            line1.set_ydata(wanted_sr)
            line2.set_xdata(xs)
            line2.set_ydata(needed_sr)
            line3.set_xdata(xs)
            line3.set_ydata(actual_sr)

            ax.relim()
            ax.autoscale_view()
            ax.set_title(f"Glide ratio: 1/{int(1/Gr)}")

            plt.draw()
            plt.pause(0.01)
        
        except Exception as e:
            print("Waiting for stream: ", e)

    def plot_altitude_error(self):
        try:
            data = self.plots["altitude_error"]
            ax = data["ax"]
            line1 = data["line1"]

            xs, altitudes, z_wanteds, Gr = self.drone_data["xs"], self.drone_data["altitude"], self.drone_data["z_wanted"], self.drone_data["Gr"]

            altitude_errors = np.array(altitudes) - np.array(z_wanteds)

            line1.set_xdata(xs)
            line1.set_ydata(altitude_errors)

            ax.relim()
            ax.autoscale_view()
            ax.set_title(f"Glide ratio: 1/{int(1/Gr)}")

            plt.draw()
            plt.pause(0.01)

        except Exception as e:
            print("Waiting for stream: ", e)
        

    def plot_filter_xy(self):
        try:
            data = self.plots["filter_xy"]
            ax = data["ax"]
            line1 = data["line1"]
            # line2 = data["line2"]

            kf_x, kf_y = self.boat_data["kf_x"], self.boat_data["kf_y"]

            line1.set_xdata(kf_x)
            line1.set_ydata(kf_y)


            # line2.set_xdata(raw_xs)
            # line2.set_ydata(raw_ys)

            ax.relim()
            ax.autoscale_view()
            ax.set_title(f"Filtered trajectory")

            plt.draw()
            plt.pause(0.01)

        except Exception as e:
            print("Waiting for stream: ", e)

    def plot_filter_latlon(self):
        try:
            data = self.plots["filter_latlon"]
            ax = data["ax"]
            line1 = data["line1"]
            # line2 = data["line2"]

            kf_lat, kf_lon = self.boat_data["kf_lat"], self.boat_data["kf_lon"]

            line1.set_xdata(kf_lat)
            line1.set_ydata(kf_lon)

            # line2.set_xdata(raw_xs)
            # line2.set_ydata(raw_ys)

            ax.relim()
            ax.autoscale_view()
            ax.set_title(f"Filtered trajectory")

            plt.draw()
            plt.pause(0.01)

        except Exception as e:
            print("Waiting for stream: ", e)

    def run(self):
        while True:
            self.drone_data = self.rc.get_latest_stream_message("drone data")[1]
            self.boat_data = self.rc.get_latest_stream_message("boat data")[1]

            self.plot_trajectory()
            # self.plot_sink_rate()
            # self.plot_altitude_error()
            # self.plot_filter_xy()
            # self.plot_filter_latlon()

            sleep(0.1)

if __name__ == "__main__":
    plotter = Plotter()
    plotter.run()
