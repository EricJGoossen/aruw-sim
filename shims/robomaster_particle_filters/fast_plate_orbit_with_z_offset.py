"""HWEMU stand-in (import-only) for robomaster_particle_filters.fast_plate_orbit_with_z_offset.

See __init__.py. The value types keep what they're given, so otto's config objects build;
ParticleFilter refuses to run.
"""


class _Values:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.__dict__.update(kwargs)

    def __repr__(self):
        return f"{type(self).__name__}(stand-in, {self.args!r})"


class InitializationPrior(_Values):
    pass


class ParticleFilterConfigurationParameters(_Values):
    pass


class Observation(_Values):
    pass


class ObservedPlate(_Values):
    pass


class ParticleFilter:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(
            "[hwemu] robomaster_particle_filters is a Jetson-only wheel; the sim's stand-in "
            "(aruw-sim shims/robomaster_particle_filters) has no particle filter"
        )
