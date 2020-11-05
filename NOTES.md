# Potential problems/improvements with the libary

### Continuous vs discrete time systems

Currently they are two different classes.
Equations of motion are often more naturally written in CT form
But a lot of other addons/attributes/logging info are more naturally written in DT form
	(eg. KF, logging state information after a DT update in the gui

Current pipeline is
CTsys = sys()
DTsys = sys.discretize()
where discretize is a method in ContinuousDynamicalSystem class.

Maybe best would be to have a single class, with a `dt` attribute.
`f` could have two behaviors, depending on whether self.dt is None or some value?
(we would still only have to implement the continuous-time _f method)
