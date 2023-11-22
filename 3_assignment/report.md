# Usage instructions
You can modify the parameters using rosparam.
IMPORTANT: If the parameters don't exist, default values are used (in each scenario, any unspecified parameter is assumed to be the default value!)

Check out load_config in config_parse.hpp for the available parameters and the default values.

Vectors are parsed with sub parameters x, y, z.
(e.g. /myvec/x, /myvec/y, /myvec/z).

# Scenarios
## Random initialization over the entire garage with small sigma_pos and no dynamic gain
### Parameters
/random_initialization true
/dynamic_sigma_pos_gain 0

### Description
The particle filter almost immediately converges to the locally best solution, which is totally wrong.
It then mis-track the forklift for the entire duration of the program.


## Random initialization over the entire garage with 10000 particles, small sigma_pos, no dynamic gain
### Parameters
/random_initialization true
/dynamic_sigma_pos_gain 0
/n_particles 10000

### Description
First thing we might try is increase the number of particles to increase the probability of a "good particle" coming up.
This way, the PF manages to initially converge to two almost equally promising solutions: one is the correct (= ground truth) one, which is later rejected, and the other is flipped 180 degrees, which is where the PF converges.
Due to the simmetry of the garage, the wrong is equally plausible and i couldn't find any easy way to fix this. In fact, until you see the plots you might even think the PF is performing well.
Additionally, even though with 10000 particles we can converge, the performance is now abysmal, taking ~300ms per iteration on my poor man's laptop.


## Random initialization over the entire garage with large sigma_pos and no dynamic gain
### Parameters
/random_initialization true
/dynamic_sigma_pos_gain 0
/sigma_pos/x 2
/sigma_pos/y 2
/sigma_pos/z 2


### Description
Another thing we might try is to increase sigma_pos so that the problem has a chance to escape the wrong attractor.
With each component of sigma_pos > 1.5, the particle filter actually converges to the (flipped) correct solution.
Unfortunately, such large sigma_pos values create a strong "vibration" effect on the resulting trajectory, greatly increasing its variance.


## Random initialization over the entire garage with small sigma_pos and dynamic gain
### Parameters
/random_initialization true

### Description
We use dynamic sigma_pos gain, which is a dynamic multiplier applied to sigma_pos based on the best particle's weight.
This allows us to converge to the correct (or flipped) solution immediately without suffering from the vibration effect or reduced performance.
We also have to greatly penalize invalid associations to take full advantage of this feature.
By reducing sigma_pos and increasing dynamic_sigma_pos_gain we improve the solution.


## Breaking everything by disabling misassociation penalties
### Parameters
/random_initialization true
/invalid_association_probability 1

### Description
Disabling misassociation penalties (setting the misassociation probability to 1) really emphasizes how important the setting is - now the least amount of associations, the higher the probability.
In fact, we see that the best particles are discarded and the worst particles are kept during resampling, resulting in a chaotic cloud of particles floating around.