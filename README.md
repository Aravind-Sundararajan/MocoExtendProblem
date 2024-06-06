[![status](https://joss.theoj.org/papers/ab4ece70adece3811308955d52be6a2f/status.svg)](https://joss.theoj.org/papers/ab4ece70adece3811308955d52be6a2f)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

![skelemen](paper/skelemens.png)

# Table of contents

- [Summary](#summary)
- [Setup](#setup)
- [Getting Started](#getting-started)
- [Testing](#testing)
- [License](#license)
- [Contributing](#contributing)
- [Code of Conduct](#code-of-conduct)

# Summary<a name="summary"></a>

MocoExtendProblem (`MEP`) is a framework to rapidly develop novel goals for biomechanical optimal control problems using OpenSim Moco and MATLAB (The Mathworks, Inc., Natick, MA, USA). `MEP` features several templates for testing and prototyping novel MocoGoals in lieu of rebuilding OpenSim or generating an .omoco file from C++ to load the problem into MATLAB. Instead, users structure custom goals, build them, and call custom goals from MATLAB scripts.

# Setup<a name="setup"></a>

- Download and install OpenSim from [SimTK](https://simtk.org) and follow the documentation for setting up OpenSim’s MATLAB scripting environment.
- Follow the instructions (OpenSim) to download necessary dependencies for both scripting in MATLAB and C++ development.
- In MATLAB, configure MEX with mex -setup C++ to use the MS VisualStudio 2019+.

# Getting Started<a name="getting-started"></a>

To create a new goal with `MEP`: 

1. OpenSim 4.5+ users should copy a goal in the `custom_goals` directory while 4.2-4.4 users  should copy a goal in `custom_goals_compat`.
2. Replace mentions of the original goal name to that of your new custom goal name in each of the 5 files and file names, being careful to also modify the include guards in the dll and register types header files. 
3. Reimplement constructProperties(), initializeOnModelImpl(), calcIntegrandImpl(), calcGoalImpl() such that they describe your custom goal.

To incorporate extend_problem goals into an existing MATLAB script, a C-style pointer to the instantiated MocoProblem is passed as a constructor argument to the `extend_problem.m` class that wraps the `MEP` MEX. Class methods of `extend_problem.m` are then used to add custom goals to the MocoProblem.

```C++
cptr = uint64(problem.getCPtr(problem));
ep = extend_problem(cptr);
ep.addMocoCustomGoal('custom_goal',weight,power,divide_by_distance);
```

# Testing<a name="testing"></a>

In the test directory, we have provided some test scripts to be run with MATLAB desktop GUI:
- test_ExtendProblem_simple.m
- example2DWalking/WalkSim_Tracking.m
- example2DWalking/WalkSim_predictive.m
- driver.m

Additionally if using an opensim version that is lower than 4.5, there are compatibility versions of `WalkSim_predictive.m` and `test_extendProblem_simple.m` to handle OpenSim version 4.2-4.4. The output of these scripts are compared against an OutputReference within the output directory. Note: you should stay on the top-level directory `MocoExtendProblem`

# License<a name="license"></a>

MEP is provided under:
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

# Contributing<a name="contributing"></a>

Please read our [contributing guidelines](CONTRIBUTING.md).

# Code of Conduct

See [CODE_OF_CONDUCT](CODE_OF_CONDUCT.md)
