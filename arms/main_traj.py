from Test import *


#def main():
 #   """Main function that runs the simulation"""
#
 #   traj = MultiAxisTrajectoryGenerator(method="cubic",
  #                                      interval=[0,10],
   #                                     ndof=1,
    #                                    start_pos=[-30],
     #                                   final_pos=[60])
    
    # generate trajectory
    #t = traj.generate(nsteps=20)

    # plot trajectory
    #raj.plot()
def main():
    """Main function that runs the simulation"""
    waypoints = [[0.2, 0.2, .2],
        [0, 0.2, .2],
        [0, 0, .2],
        [0.2, 0, .2]]
    traj = MultiAxisTrajectoryGenerator(
        method="quintic",     # Smoother (continuous acceleration)
        interval=[0, 8],      # Total time
        ndof=2,              # x and y axes
        waypoints=waypoints,
        time_alloc="distance" # Time proportional to segment length
    )

    traj.generate(nsteps=200)
    traj.plot()

if __name__ == "__main__":
    main()