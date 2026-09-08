"""BVH lifecycle wrapper for Newton ray sensors."""


class NewtonBvh:
    def __init__(self, model, state):
        self.model = model
        self.has_particles = getattr(model, "particle_q", None) is not None and model.particle_q.shape[0] > 0
        # BVHs are built by ModelBuilder.finalize() in Newton >= 1.5; just refit to the current state.
        self.refit(state)

    def refit(self, state):
        self.model.bvh_refit_shapes(state)
        if self.has_particles:
            self.model.bvh_refit_particles(state)
