import matplotlib.pyplot as plt
from redis_communication import RedisClient
from time import sleep

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
        ax_sr.set_xlabel("Distance since descent start (m)")
        ax_sr.set_ylabel("Sink rate (m/s)")
        ax_sr.legend()

        self.plots["sink rate"] = {
            "fig": fig_sr,
            "ax": ax_sr,
            "line1": line1_sr,
            "line2": line2_sr 
        }

    def plot_trajectory(self):
        try:
            data = self.plots["trajectory"]
            ax = data["ax"]
            line1 = data["line1"]
            line2 = data["line2"]

            # [xs, altitudes, boat_altitude, z_wanteds] = self.rc.get_latest_stream_message("traj_plot")[1]
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
            data = self.plots["sink rate"]
            ax = data["ax"]
            line1 = data["line1"]
            line2 = data["line2"]

            xs, wanted_sr, needed_sr, Gr = self.drone_data["xs"], self.drone_data["wanted_sr"], self.drone_data["needed_sr"], self.drone_data["Gr"]

            line1.set_xdata(xs)
            line1.set_ydata(wanted_sr)
            line2.set_xdata(xs)
            line2.set_ydata(needed_sr)

            ax.relim()
            ax.autoscale_view()
            ax.set_title(f"Glide ratio: 1/{int(1/Gr)}")

            plt.draw()
            plt.pause(0.01)
        
        except Exception as e:
            print("Waiting for stream", e)
        

    def run(self):
        while True:
            self.drone_data = self.rc.get_latest_stream_message("drone data")[1]
            self.boat_data = self.rc.get_latest_stream_message("boat data")[1]

            self.plot_trajectory()
            self.plot_sink_rate()

            sleep(1)


if __name__ == "__main__":
    plotter = Plotter()
    plotter.run()
