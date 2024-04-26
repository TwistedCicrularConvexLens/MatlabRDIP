# MatlabRDIP
This repository was created to demonstrate the control of an Rotary Double Inverted Pendulum.

The main notable files are:

(Linearization.mlx)

Which is needed to run the later systems that use the linerized matricies. It also contains the full non-liner dynamics.

(MultibodySimBasic.slx)

The non-control version of the RDIP. Used to compare between symbollic toolbox and simscape multibody.

(StateSpaceControl.slx)

The linerized control acting on the linerized plant. (Run Linearization.mlx before running this)

(Animation3.m)

The script used for plotting and comparing the angles between the two simulations. (Run MultibodySimBasic.slx before running this.)