Installation 
============
*occ_airconics* accesses the powerful [Open CASCADE](http://www.opencascade.com/) geometry kernel through the Python-OCC package. It is possible to build [pythonocc-core](https://github.com/tpaviot/pythonocc-core) from source by [following their installation instructions](https://github.com/tpaviot/pythonocc-core/blob/master/INSTALL.md), however a convenient alternative is to use the prebuilt conda packages suitable for win32/win64/osx64/linux64 users.

Note that *occ_airconics* is not currently available through PyPI.


Conda packages
--------------

*pythonocc-core* is listed as a dependency of *occ_airconics*, therefore users should simply add the appropriate conda channels to their `~/.condarc` file:

	conda config --add channels dlr-sc     # the pythonocc-core channel

	conda config --add channels prchambers # the occ_airconics channel

Or do this manually by editing their `~/.condarc` contents, e.g.:

	channels:
	    - https://conda.anaconda.org/dlr-sc
	    - https://conda.anaconda.org/prchambers
		- defaults

Then install occ_airconics via

	conda install occ_airconics

And that's it! *pythonocc-core* will be installed automatically.


Installation from source
------------------------

Obtain and build a copy of *pythonocc-core* from [GitHub](https://github.com/tpaviot/pythonocc-core) following their instructions.

Then clone *occ_airconics* from [GitHub](https://github.com/p-chambers/occ_airconics) with:

	git clone https://github.com/p-chambers/occ_airconics

And install with

	cd occ_airconics
	python setup.py install

Or

	pip install occ_airconics

Developers should also add the develop flag, i.e.

	python setup.py install develop
