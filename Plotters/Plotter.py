from matplotlib import animation
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth

if __name__ == "__main__":
   # open data.csv
   csvFile = open("data.csv", "r", newline='', encoding='utf-8')
   csvData = pd.read_csv(csvFile)

   # time data
   times = csvData["Time"].to_list()

   # ------------------------------------------------

   # agent_0 position data
   agent_0_pos_x = csvData["agent_0_pos_x"].to_list()
   agent_0_pos_y = csvData["agent_0_pos_y"].to_list()
   agent_0_pos_z = csvData["agent_0_pos_z"].to_list()

   # agent_1 position data
   agent_1_pos_x = csvData["agent_1_pos_x"].to_list()
   agent_1_pos_y = csvData["agent_1_pos_y"].to_list()
   agent_1_pos_z = csvData["agent_1_pos_z"].to_list()

   # agent_2 position data
   agent_2_pos_x = csvData["agent_2_pos_x"].to_list()
   agent_2_pos_y = csvData["agent_2_pos_y"].to_list()
   agent_2_pos_z = csvData["agent_2_pos_z"].to_list()

   # ------------------------------------------------

   # agent_0 velocity data
   agent_0_vel_x = csvData["agent_0_vel_x"].to_list()
   agent_0_vel_y = csvData["agent_0_vel_y"].to_list()
   agent_0_vel_z = csvData["agent_0_vel_z"].to_list()
   agent_0_vel_force = csvData["agent_0_vel_force"].to_list()
   
   # agent_1 velocity data
   agent_1_vel_x = csvData["agent_1_vel_x"].to_list()
   agent_1_vel_y = csvData["agent_1_vel_y"].to_list()
   agent_1_vel_z = csvData["agent_1_vel_z"].to_list()
   agent_1_vel_force = csvData["agent_0_vel_force"].to_list()

   # agent_2 velocity data
   agent_2_vel_x = csvData["agent_2_vel_x"].to_list()
   agent_2_vel_y = csvData["agent_2_vel_y"].to_list()
   agent_2_vel_z = csvData["agent_2_vel_z"].to_list()
   agent_2_vel_force = csvData["agent_0_vel_force"].to_list()

   # ------------------------------------------------

   # agent_0 formation data
   agent_0_formation_control_x = csvData["agent_0_formation_control_x"].to_list()
   agent_0_formation_control_y = csvData["agent_0_formation_control_y"].to_list()
   agent_0_formation_control_z = csvData["agent_0_formation_control_z"].to_list()
   agent_0_formation_control_force = csvData["agent_0_formation_control_force"].to_list()

   # agent_1 formation data
   agent_1_formation_control_x = csvData["agent_1_formation_control_x"].to_list()
   agent_1_formation_control_y = csvData["agent_1_formation_control_y"].to_list()
   agent_1_formation_control_z = csvData["agent_1_formation_control_z"].to_list()
   agent_1_formation_control_force = csvData["agent_1_formation_control_force"].to_list()

   # agent_2 formation data
   agent_2_formation_control_x = csvData["agent_2_formation_control_x"].to_list()
   agent_2_formation_control_y = csvData["agent_2_formation_control_y"].to_list()
   agent_2_formation_control_z = csvData["agent_2_formation_control_z"].to_list()
   agent_2_formation_control_force = csvData["agent_2_formation_control_force"].to_list()

   # ------------------------------------------------

   # agent_0 formation data
   agent_0_avoidance_control_x = csvData["agent_0_avoidance_control_x"].to_list()
   agent_0_avoidance_control_y = csvData["agent_0_avoidance_control_y"].to_list()
   agent_0_avoidance_control_z = csvData["agent_0_avoidance_control_z"].to_list()
   agent_0_avoidance_control_force = csvData["agent_0_avoidance_control_force"].to_list()

   # agent_1 formation data
   agent_1_avoidance_control_x = csvData["agent_1_avoidance_control_x"].to_list()
   agent_1_avoidance_control_y = csvData["agent_1_avoidance_control_y"].to_list()
   agent_1_avoidance_control_z = csvData["agent_1_avoidance_control_z"].to_list()
   agent_1_avoidance_control_force = csvData["agent_1_avoidance_control_force"].to_list()

   # agent_2 formation data
   agent_2_avoidance_control_x = csvData["agent_2_avoidance_control_x"].to_list()
   agent_2_avoidance_control_y = csvData["agent_2_avoidance_control_y"].to_list()
   agent_2_avoidance_control_z = csvData["agent_2_avoidance_control_z"].to_list()
   agent_2_avoidance_control_force = csvData["agent_2_avoidance_control_force"].to_list()
   
   # ------------------------------------------------

   # agent_0 trajectory data
   agent_0_trajectory_control_x = csvData["agent_0_trajectory_control_x"].to_list()
   agent_0_trajectory_control_y = csvData["agent_0_trajectory_control_y"].to_list()
   agent_0_trajectory_control_z = csvData["agent_0_trajectory_control_z"].to_list()
   agent_0_trajectory_control_force = csvData["agent_0_trajectory_control_force"].to_list()

   # agent_1 trajectory data
   agent_1_trajectory_control_x = csvData["agent_1_trajectory_control_x"].to_list()
   agent_1_trajectory_control_y = csvData["agent_1_trajectory_control_y"].to_list()
   agent_1_trajectory_control_z = csvData["agent_1_trajectory_control_z"].to_list()
   agent_1_trajectory_control_force = csvData["agent_1_trajectory_control_force"].to_list()

   # agent_2 trajectory data
   agent_2_trajectory_control_x = csvData["agent_2_trajectory_control_x"].to_list()
   agent_2_trajectory_control_y = csvData["agent_2_trajectory_control_y"].to_list()
   agent_2_trajectory_control_z = csvData["agent_2_trajectory_control_z"].to_list()
   agent_2_trajectory_control_force = csvData["agent_2_trajectory_control_force"].to_list()


   # 2d plot of heights (y-axis) against time (x-axis)
   fig, ax = plt.subplots()
   
   agent_0_pos_z = [(i*-1.0 + 1.718) for i in agent_0_pos_z]
   ax.plot(times, agent_0_pos_z, color = 'red', label = 'Agent_0')

   agent_1_pos_z = [(i*-1.0 + 1.718) for i in agent_1_pos_z]
   ax.plot(times, agent_1_pos_z, color = 'green', label = 'Agent_1')

   agent_2_pos_z = [(i*-1.0 + 1.718) for i in agent_2_pos_z]
   ax.plot(times, agent_2_pos_z, color = 'blue', label = 'Agent_2')

   ax.legend(loc = 'upper left')
   ax.set(xlabel='time (s)', ylabel='height (m)',
      title='Height of Agents')
   ax.grid()
   ax.set_ylim([0.0, 4.0])

   fig.savefig('./Plotters/Height_of_Agents.png')
   
   
   # 2d plot of avoidance force (y-axis) againts time (x-axis)
   fig1, ax1 = plt.subplots()
   
   ax1.plot(times, agent_0_avoidance_control_force, color = 'red', label = 'Agent_0')
   ax1.plot(times, agent_1_avoidance_control_force, color = 'green', label = 'Agent_1')
   ax1.plot(times, agent_2_avoidance_control_force, color = 'blue', label = 'Agent_2')

   ax1.legend(loc = 'upper left')
   ax1.set(xlabel='time (s)', ylabel='avoidance force (m/s)',
      title='Avoidance Force of Agents')
   ax1.grid()
   ax1.set_ylim([0.0, 1.0])
   ax1.set_xlim([4.0, 8.0])

   fig1.savefig('./Plotters/Avoidance_Force_of_Agents.png')

   # 2d plot of formation force (y-axis) againts time (x-axis)
   fig2, ax2 = plt.subplots()
   
   ax2.plot(times, agent_0_formation_control_force, color = 'red', label = 'Agent_0')
   ax2.plot(times, agent_1_formation_control_force, color = 'green', label = 'Agent_1')
   ax2.plot(times, agent_2_formation_control_force, color = 'blue', label = 'Agent_2')

   ax2.legend(loc = 'upper left')
   ax2.set(xlabel='time (s)', ylabel='formation force (m/s)',
      title='Formation Force of Agents')
   ax2.grid()

   ax2.set_ylim([0.0, 2.0])
   fig2.savefig('./Plotters/Formation_Force_of_Agents.png')

   # 2d plot of trajectory force (y-axis) againts time (x-axis)
   fig3, ax3 = plt.subplots()
   
   ax3.plot(times, agent_0_trajectory_control_force, color = 'red', label = 'Agent_0')
   ax3.plot(times, agent_1_trajectory_control_force, color = 'green', label = 'Agent_1')
   ax3.plot(times, agent_2_trajectory_control_force, color = 'blue', label = 'Agent_2')

   ax3.legend(loc = 'upper left')
   ax3.set(xlabel='time (s)', ylabel='trajectory force (m/s)',
      title='Trajectory Force of Agents')
   ax3.grid()

   ax3.set_ylim([0.0, 3.5])
   fig3.savefig('./Plotters/Trajectory_Force_of_Agents.png')

   # 2d plot of velocity (y-axis) againts time (x-axis)
   fig4, ax4 = plt.subplots()

   x = np.array(times)

   y = np.array(agent_0_vel_force) 
   ax4.plot(x, smooth(y, 23), color = 'red', label = 'Agent_0')

   y = np.array(agent_1_vel_force) 
   ax4.plot(times, smooth(y, 27), color = 'green', label = 'Agent_1')

   y = np.array(agent_2_vel_force)
   ax4.plot(times, smooth(y, 31), color = 'blue', label = 'Agent_2')

   ax4.legend(loc = 'upper left')
   ax4.set(xlabel='time (s)', ylabel='velocity (m/s)',
      title='Velocity of Agents')
   ax4.grid()

   # ax3.set_ylim([0.0, 2.0])
   fig4.savefig('./Plotters/Velocity_of_Agents.png')

   # 2d plot of coordinates
   fig5, ax5 = plt.subplots()

   x = np.array(agent_0_pos_x[:-6])
   y = np.array(agent_0_pos_y[:-6])
   x_mean_0 = x.reshape(-1, 25).mean(axis=1)
   y_mean_0 = y.reshape(-1, 25).mean(axis=1)

   x = np.array(agent_1_pos_x[:-6])
   y = np.array(agent_1_pos_y[:-6])
   x_mean_1 = x.reshape(-1, 25).mean(axis=1)
   y_mean_1 = y.reshape(-1, 25).mean(axis=1)

   x = np.array(agent_2_pos_x[:-6])
   y = np.array(agent_2_pos_y[:-6])
   x_mean_2 = x.reshape(-1, 25).mean(axis=1)
   y_mean_2 = y.reshape(-1, 25).mean(axis=1)

   ax5.legend(["Agent_0", "Agent_1", "Agent_2"])

   animate_0_x = []
   animate_0_y = []
   animate_1_x = []
   animate_1_y = []
   animate_2_x = []
   animate_2_y = []

   def update_plot(
      i, ax, x_mean_0, y_mean_0, x_mean_1, y_mean_1, x_mean_2, y_mean_2):
      ax.cla()

      animate_0_x.append(y_mean_0[i])
      animate_0_y.append(x_mean_0[i])

      animate_1_x.append(y_mean_1[i])
      animate_1_y.append(x_mean_1[i])

      animate_2_x.append(y_mean_2[i])
      animate_2_y.append(x_mean_2[i])

      # draw the dots
      for iter in range(len(animate_0_x)):
         ax.scatter(animate_0_x[iter], animate_0_y[iter], s=2, color='red', label='Agent_0')
      for iter in range(len(animate_1_x)):
         ax.scatter(animate_1_x[iter], animate_1_y[iter], s=2, color='green', label='Agent_1')
      for iter in range(len(animate_2_x)):
         ax.scatter(animate_2_x[iter], animate_2_y[iter], s=2, color='blue', label='Agent_2')

      # draw the triangle
      side_1_x, side_1_y = [animate_0_x[-1:], animate_1_x[-1:]], [animate_0_y[-1:], animate_1_y[-1:]]
      side_2_x, side_2_y = [animate_0_x[-1:], animate_2_x[-1:]], [animate_0_y[-1:], animate_2_y[-1:]]
      side_3_x, side_3_y = [animate_1_x[-1:], animate_2_x[-1:]], [animate_1_y[-1:], animate_2_y[-1:]]

      ax.plot(side_1_x, side_1_y, side_2_x, side_2_y, side_3_x, side_3_y, color='gray', alpha=0.5, label="SWARM")

      ax.set_ylim([-2.0, 25.0])
      ax.set_xlim([-2.0, 25.0])
      ax.set(xlabel='x (m)', ylabel='y (m)',
         title='An Example Scenario of SWARM')
      ax.grid()
      ax.legend(title="Agents", loc='upper right')
      ax.legend(["Agent_0", "Agent_1", "Agent_2"])
      leg = ax.get_legend()
      leg.legendHandles[0].set_color('red')
      leg.legendHandles[1].set_color('green')
      leg.legendHandles[2].set_color('blue')

   anim = FuncAnimation(
      fig5,
      update_plot,
      fargs=(ax5, x_mean_0, y_mean_0, x_mean_1, y_mean_1, x_mean_2, y_mean_2),
      frames=100,
      interval=16.4
   )

   plt.show()

   mywriter = animation.FFMpegWriter(fps=60)
   anim.save('myanimation.mp4', writer=mywriter)
