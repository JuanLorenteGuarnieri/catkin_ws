# Laboratory 2-3


`p14_arob_lab3_drones/src/drone_race.cpp`

In the drawGateMarkers_ function, each gate is created by defining multiple Marker objects that represent the corners and edges of the gate. 
Each marker is given specific properties, such as shape, color, and position.


```c++
void DroneRace::generateTrajectory_() {...}
```

The starting position is set to the origin (0.0, 0.0, 0.0).

For each gate in gates_, an intermediate Vertex is added to represent the waypoint at that gate's position.
The position of each gate is added as a constraint to ensure that the trajectory passes through each gate.
Each gate's orientation is used to calculate a direction vector perpendicular to the gate. A velocity constraint is added along this direction with a desired magnitude, ensuring that the drone approaches each gate at a controlled speed and in the correct direction.

After passing through all the gates, the drone returns to the origin as the final point.

We compute the estimated time required for each segment between the vertices. This considers constraints on maximum velocity and maximum acceleration


![Gates 3](images/p3/gates3.png)

_Easy Gates_

![Gates 4](images/p3/gates4.png)

_Easy Gates_

![Gates 5](images/p3/gates5.png) 

_Hard Gates_ 

![Gates 6](images/p3/gates6.png) 

_Hard Gates_