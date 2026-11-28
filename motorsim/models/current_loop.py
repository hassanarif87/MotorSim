"""Convenience wrapper for the standard fixed-current drive wiring."""

from .drive import DriveModel


def linearize_current_loop(config):
    """Return the sampled iq_ref-to-iq model and operating-point data."""
    system, model = DriveModel(config).linearize("current_reference.value", "current_feedback.iq")
    system.set_inputs(["iq_ref"])
    system.set_outputs(["iq"])
    model["iq_ref"] = model["input_value"]
    return system, model
