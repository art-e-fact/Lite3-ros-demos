"""BVH lifecycle wrapper for Newton ray sensors."""

import newton


class NewtonBvh:
    def __init__(self, model, state):
        self.model = model
        self.has_particles = getattr(model, "particle_q", None) is not None and model.particle_q.shape[0] > 0
        newton.geometry.build_bvh_shape(model, state)
        if self.has_particles:
            newton.geometry.build_bvh_particle(model, state)

    def refit(self, state):
        newton.geometry.refit_bvh_shape(self.model, state)
        if self.has_particles:
            newton.geometry.refit_bvh_particle(self.model, state)
