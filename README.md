# MuJoCo Bootcamp W04 - Energy

[Here](https://www.youtube.com/watch?v=HYVPysAGp6g&list=PLc7bpbeTIk758Ad3fkSywdxHWpBh9PM0G&index=9) is the video.

Again, rather than using the provided template, I used the `basic.cc` from the MuJoCo distribution.

This section is about measuring energy of the system and using it to stabilize a double pendulum.

### Building
This is a standard CMake project and I normally run it from CLion.

[CMakeLists.txt](CMakeLists.txt) has hard-coded path to MuJoCo - you will need to update it first to your path.

To manually build & run:

    mkdir build
    cd build
    cmake ..
    build -j4

    ./w04

### Measuring Energy
MuJoCo can calculate both potential and kinetic energy of the system. It can be accessed in the controller and used for calculation.

The energy calculation needs to be enabled in the XML file (at the top):

	<option timestep="0.001"  integrator="RK4">
		<flag sensornoise="enable" energy="enable" contact="enable" />
	</option>

This also sets a smaller timestamp and changes the integrator to RK4. Both of these are necessary for more precise calculations to keep the energy from dissipating.
You can try running the double pendulum with default settings and you should see total energy dropping fairly quickly.

In some cases it may be also necessary to turn of `contact` (collisions) to reduce the energy lost to friction.

Once enabled, the energy can be calculated by calling (in the controller, i.e. at each step):

    mj_energyPos(m, d);
    mj_energyVel(m, d);

And then it can be accessed like this:

    std::cout << d->time << " potential energy: " << d->energy[0] << " kinetic energy: " << d->energy[1] << " total: " << d->energy[0] + d->energy[1] << std::endl;

The first index of the `d->energy` is the potential energy and the second is the kinetic energy ([see the details](https://mujoco.readthedocs.io/en/latest/APIreference.html#mjdata) of `mjData`).

### Stabilizing the Pendulum
See the controller `void pendulumController(const mjModel* m, mjData* d) {` (line 102).

The first part stabilizes the pendulum and the second should do PD control to move the pendulum to a different location, but it doesn't seem to work.
