The Lab-4 Behavioral Motion Control System

In this lab I have considered spheres as the objects and predators in the scene.
The input:
1. Data like number of objects(NUM_SPHERES) and dt = 0.016(change dt as per requirement) 
2.mass
3.radius of sphere(radius)
4.restitution
5.floorLevel
6.groundFriction
7.moment of Inertia is calculated on formula I=2/5 m r^2 which is for solid sphere
The above values are taken static can be modified if needed. 
8.others like Linear velocity,Position,Angular velocity,Inital oriention(roll,pitch,yaw) are taken random using rand() func with in defined intervals.
9.for more accurate display of animation I have made use of color component which RGB values that are generated random for each rigidbody

I have tried to flock with predator and when the objects comes closer within in distance one of the object is reswapned 
whenever object touches the predator the predator will take new psition randomly and 
still dt is dependent on frametime but dt=0.016 is taken stand for making a film(24 fps). you can try different dt.
