# particle_filter

<p>Particle filter is used to track the catheter.</p>

## Youtube
<p>locate catheter with known input: https://www.youtube.com/watch?v=ox7m-5A8HFE</p>
<p>locate catheter without known input: https://www.youtube.com/watch?v=E5JJJG2jPQs</p>
<p>__Tracking catheter with incremental motion model__: https://www.youtube.com/watch?v=vmrypCFaFNg</p>

## CatheterPose

<p>A struct data type CatheterPose is defined in the header file, ParticleFilter.h. It contains three points (A, B, C) and one tangent vetor (t). It provides two functions and overloads three operators.</p> 

<p>The function getBSpline(s) is the mapping, g:[0,1] -> R^3 </p>
<p>It gives user access to obtaining the point in the combined Bezier spline.</p>
<p>When s belongs to [0,0.5], it returns the point on the cubic Bezier spline.</p>
<p>When s belongs to [0.5,0], it returns the point on the quadratic Bezier spline.</p>

## ParticleFilter

<p>Constructors:</p>
<p>It provides users two ways to construct the ParticleFilter. The first requires the number of particles and the current (amps) in the axial coil. Then users can generate the particles based on the current (amps) and number of the particles. See test_main.cpp. The other requires a range of currents, its step and the number of particles for each step. It generates particles based on the range of currents. See test_main2.cpp</p>
<p>test_main3.cpp gives an example of the usage of incremental model.</p>

<p>The process:</p>
<p>(1) Initialize the object, ParticleFilter</p>
<p>(2) Set the motion model's standard variances</p>
<p>(3) Generate particles</p>
<p>(4) Get the particles from motion model and catheter pose from measurement model</p>
<p>(5) Compare these particles with the one from measurement, and compute the corresponding weights</p>
<p>(6) Set the weights</p>
<p>(7) Resample</p>
<p>(8) Repeat (4)~(7) until the desired variances of the particles are encountered or the number of number of particles reaches the minimum threshold. </p>

## Resample
<p>The ParticleFilter provides two ways to resample, resample() and resample2()</p>
<p>resample() can resample the particles without reducing the number of the particles.</p>
<p>resample2() will delete the particles that have very small weights.</p>
<p>resample2() is more efficient than resample(), but it will lose some samples for each iteration.</p>
<p>resample2() is commonly used in the stationary motion model like test_main.cpp and test_main2.cpp, while resample() is used in the incremental model like test_main3.cpp</p>


## Usages
<p>Tracking catheter based on certian current (amps):</p>
<p>roslaunch particle_filter known_amps.launch --screen</p>
<p>rosrun catheter_sim amps_emitter</p>

<p>Tracking catheter without known current (amps):</p>
<p>roslaunch particle_filter unknown_amps.launch --screen</p>
<p>rosrun catheter_sim amps_emitter</p>

<p>Tracking catheter with the incremental model:</p>
<p>roslaunch particle_filter incremental_model.launch --screen</p>
<p>Or:</p>
<p>roslaunch particle_filter incremental_model2.launch --screen</p>


