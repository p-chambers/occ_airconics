# OCC_AirCONICS
Aircraft Configuration through Integrated Cross-disciplinary Scripting, Python standalone package built on PythonOCC

This repository contains the source code for a standalone version of the AirCONICS* Rhino plugin.

For full documentation, refer to the [occ_airconics website](http://occ-airconics.readthedocs.io/en/latest/index.html).

## Installation

### Conda packages

*pythonocc-core* is listed as a dependency of *occ_airconics*, therefore users should simply add the appropriate conda channels to their `~/.condarc` file:

	conda config --add channels dlr-sc     # the pythonocc-core channel

	conda config --add channels prchambers # the occ_airconics channel

Or do this manually by editing their `~/.condarc` contents, e.g.:

	channels:
	    - https://conda.anaconda.org/dlr-sc
	    - https://conda.anaconda.org/prchambers
		- defaults

Then install *occ_airconics* via

	conda install occ_airconics

And that's it! *pythonocc-core* will be installed automatically.


### Installation from source

Obtain and build a copy of *pythonocc-core* from [GitHub](https://github.com/tpaviot/pythonocc-core) following their instructions.

Then clone *occ_airconics* from [GitHub](https://github.com/p-chambers/occ_airconics) with:

	git clone https://github.com/p-chambers/occ_airconics

And install with

	cd occ_airconics
	python setup.py install


## Examples
Examples can be found in the ipython notebook in the 'examples' directory.

*AirCONICS is a Rhino Aircraft geometry plugin created by Andras Sobester, see the following links for further information:
- https://github.com/sobester/AirCONICS
- https://aircraftgeometrycodes.wordpress.com/
