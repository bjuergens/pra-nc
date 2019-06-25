# encoding: utf8
"""
A very simple example of using STDP.

A single postsynaptic neuron fires at a constant rate. We connect several
presynaptic neurons to it, each of which fires spikes with a fixed time
lag or time advance with respect to the postsynaptic neuron.
The weights of these connections are very small, so they will not
significantly affect the firing times of the post-synaptic neuron.
We plot the amount of potentiation or depression of each synapse as a
function of the time difference.


Usage: python simple_STDP.py [-h] [--plot-figure] [--debug DEBUG] simulator

positional arguments:
  simulator      neuron, nest, brian or another backend simulator

optional arguments:
  -h, --help     show this help message and exit
  --plot-figure  Plot the simulation results to a file
  --fit-curve    Calculate the best-fit curve to the weight-delta_t measurements
  --debug DEBUG  Print debugging information

"""

from __future__ import division
from math import exp
import numpy
import neo
from quantities import ms
from pyNN.utility import get_simulator, init_logging, normalized_filename
from pyNN.utility.plotting import DataTable
from pyNN.parameters import Sequence

from braitenberg import create_brain
brain, proj1, proj2, proj3, proj4, proj5, proj6 = create_brain()



# === Configure the simulator ===============================================

sim, options = get_simulator(("--plot-figure", "Plot the simulation results to a file", {"action": "store_true"}),
                             ("--fit-curve", "Calculate the best-fit curve to the weight-delta_t measurements", {"action": "store_true"}),
                             ("--dendritic-delay-fraction", "What fraction of the total transmission delay is due to dendritic propagation", {"default": 1}),
                             ("--debug", "Print debugging information"))

if options.debug:
    init_logging(None, debug=True)

sim.setup(timestep=1, min_delay=1, max_delay=3)
t_stop = 30
brain.record('spikes')
# brain.record(['spikes', 'v'])

for a in range(200):
    print(a)
    sim.run(t_stop)

sim.end()

print("end reached")