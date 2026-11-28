"""Sampled input/output analysis using the native drive and scheduler."""

import control as ct
import numpy as np


class DriveModel:
    """Probe a configured C++ drive and identify its local sampled response."""

    def __init__(self, config):
        from motorsim import _pmsmcpp

        self._drive = _pmsmcpp.DriveProbe(config)

    @property
    def input_ports(self):
        """Names of reference and disturbance ports that can be perturbed."""
        return tuple(self._drive.input_ports)

    @property
    def output_ports(self):
        """Names of configured numeric log ports and writable source ports."""
        return tuple(self._drive.output_ports)

    @property
    def sample_time(self):
        """Time in seconds between matching phases of the drive schedule."""
        return self._drive.sample_time

    @property
    def tick_time(self):
        """Duration of one C++ integration tick in seconds."""
        return self._drive.tick_time

    @property
    def sample_ticks(self):
        """Number of integration ticks between matching scheduler phases."""
        return self._drive.sample_ticks

    def run(self, output_ports, *, samples, start_tick=0, sample_every=None, changes=()):
        """Run a fresh drive and return (time_s, values).

        Rows are samples and columns follow output_ports. Each change is a
        (tick, writable_port, absolute_value) tuple. Changes are applied before
        the drive callbacks, and outputs are recorded after them. At the same
        tick, changes are applied in the order supplied.
        """
        outputs = [output_ports] if isinstance(output_ports, str) else list(output_ports)
        if sample_every is None:
            sample_every = self.sample_ticks
        return self._drive.run(outputs, start_tick, samples, sample_every, list(changes))

    def linearize(
        self, input_port, output_port, *, delta=1e-3, settle_samples=1024,
        samples=256, rcond=1e-7,
    ):
        """Return a local sampled model and operating-point data.

        Pass a port name or a sequence of names for each argument. A sequence
        of perturbation amplitudes selects a different delta for each input.
        The identified states describe modes visible through the chosen ports.
        """
        single_input = isinstance(input_port, str)
        single_output = isinstance(output_port, str)
        inputs = [input_port] if single_input else list(input_port)
        outputs = [output_port] if single_output else list(output_port)
        if not inputs or len(set(inputs)) != len(inputs):
            raise ValueError("Expected distinct input ports")
        if not outputs or len(set(outputs)) != len(outputs):
            raise ValueError("Expected distinct output ports")
        for port in inputs:
            if port not in self.input_ports:
                raise ValueError(f"Unknown writable input port: {port}")
        if samples < 8 or settle_samples < 1:
            raise ValueError("Expected at least eight samples and a positive settling interval")
        if not 0 < rcond < 1:
            raise ValueError("rcond must be between 0 and 1")
        deltas = np.broadcast_to(np.asarray(delta, dtype=float), (len(inputs),))
        if not np.isfinite(deltas).all() or np.any(deltas <= 0):
            raise ValueError("Perturbations must be finite and positive")

        start_tick = self.sample_ticks * settle_samples
        _, trace = self.run(inputs + outputs, samples=samples, start_tick=start_tick)
        input_values = trace[0, :len(inputs)]
        baseline = trace[:, len(inputs):]
        if not np.isfinite(trace).all():
            raise ValueError("Drive response contains nonfinite samples")

        step = np.empty((samples, len(outputs), len(inputs)))
        for i, (port, amplitude, value) in enumerate(zip(inputs, deltas, input_values)):
            lower, upper = value - amplitude, value + amplitude
            if not np.isfinite([lower, upper]).all() or not lower < value < upper:
                raise ValueError(f"Perturbation must change {port} to finite values")
            _, plus = self.run(outputs, samples=samples, start_tick=start_tick,
                               changes=[(start_tick, port, upper)])
            _, minus = self.run(outputs, samples=samples, start_tick=start_tick,
                                changes=[(start_tick, port, lower)])
            if not np.isfinite([plus, minus]).all():
                raise ValueError("Drive response contains nonfinite samples")

            signal = np.max(np.abs(plus - minus), axis=0)
            tolerance = 1e-10 + 1e-5 * signal
            if np.any(np.max(np.abs(baseline - baseline[0]), axis=0) > tolerance):
                raise ValueError("Baseline has not settled at the sampled schedule phase")
            if np.any(np.max(np.abs(plus + minus - 2 * baseline), axis=0) > tolerance):
                raise ValueError(f"Perturbations of {port} do not give a symmetric small-signal response")
            step[:, :, i] = (plus - minus) / (upper - lower)

        A, B, C, D = _realize_step(step, rcond)
        system = ct.ss(A, B, C, D, self.sample_time, inputs=inputs, outputs=outputs)
        return system, {
            "A": A, "B": B, "C": C, "D": D, "dt": self.sample_time,
            "input_value": input_values[0] if single_input else input_values,
            "output_value": baseline[0, 0] if single_output else baseline[0],
            "input_port": input_port, "output_port": output_port,
        }


def _realize_step(step, rcond):
    """Realize the Markov parameters using a shifted Hankel matrix and SVD."""
    markov = np.diff(step, axis=0)
    noutputs, ninputs = step.shape[1:]
    width = min(64, len(markov) // 2)
    indices = np.add.outer(np.arange(width), np.arange(width))
    h0 = markov[indices].transpose(0, 2, 1, 3).reshape(width * noutputs, width * ninputs)
    h1 = markov[indices + 1].transpose(0, 2, 1, 3).reshape(h0.shape)
    left, singular, right = np.linalg.svd(h0, full_matrices=False)
    rank = np.count_nonzero(singular > max(rcond * singular[0], 1e-10))
    D = step[0].copy()

    left = left[:, :rank]
    right = right[:rank]
    root = np.sqrt(singular[:rank])
    A = (left.T @ h1 @ right.T) / root[:, None] / root[None, :]
    B = root[:, None] * right[:, :ninputs]
    C = left[:noutputs] * root

    state = np.zeros_like(B)
    predicted = np.empty_like(step)
    for i in range(len(step)):
        predicted[i] = C @ state + D
        state = A @ state + B
    if not np.isfinite(predicted).all() or not np.allclose(
        predicted, step, rtol=1e-4, atol=1e-8 * np.maximum(1.0, np.max(np.abs(step), axis=0)),
    ):
        raise ValueError("The realized model does not reproduce the measured step responses")
    return A, B, C, D
