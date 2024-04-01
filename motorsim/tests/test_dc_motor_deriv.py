import numpy as np
import pytest
from motorsim.models.dc_models import dc_motor_deriv

@pytest.fixture
def motor_params():
    return {
        'Kt': 0.1,
        'v_dc': 10,
        'Lm': 0.03,
        'Rm': 0.03,
        'J': 0.03,
        'b': 0.1  # Adjust the viscous friction for testing
    }

def test_dc_motor_deriv(motor_params):
    t = 0.0  # Time
    x = np.array([0.0, 0.0])  # Initial states
    u = np.array([0.5, 1.0])  # Inputs
    expected_result = np.array([0.0, 0.33333333])  # Expected derivative of state vector

    result = dc_motor_deriv(t, x, u, motor_params)

    # Compare the result with the expected value
    np.testing.assert_allclose(result, expected_result)

# Additional test cases can be added for different scenarios

def test_dc_motor_deriv_no_load(motor_params):
    t = 0.0
    x = np.array([0.0, 0.0])
    u = np.array([0.5, 0.0])  # No load on the motor
    expected_result = np.array([0.0, 0.0])  # No change in rotational speed

    result = dc_motor_deriv(t, x, u, motor_params)

    np.testing.assert_allclose(result, expected_result)

# Run tests
if __name__ == "__main__":
    pytest.main()
