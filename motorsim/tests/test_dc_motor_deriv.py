import numpy as np
import pytest
from motorsim.models.dc_models import dc_motor_deriv

@pytest.fixture
def motor_params():
    return {
        'Kt': 0.1,
        'v_dc': 10,
        'Lm': 0.05,
        'Rm': 1,
        'J': 0.01,
        'b': 0.0  # Adjust the viscous friction for testing
    }


# Additional test cases can be added for different scenarios

def test_dc_motor_deriv_no_load(motor_params):
    t = 0.0
    x = np.array([0.0, 0.0])
    u = np.array([0.1, 0.0])  # No load on the motor
    expected_result = np.array([20.0, 0.0])  # No change in rotational speed

    result = dc_motor_deriv(t, x, u, motor_params)
    np.testing.assert_allclose(result, expected_result)

def test_dc_motor_deriv_load(motor_params):
    t = 0.0  # Time
    x = np.array([0.0, 0.0])  # Initial states [omega, i_m]
    u = np.array([0.5, 1.0])  # Inputs
    expected_result = np.array([ 100., -100.])  # Expected derivative of state vector

    result = dc_motor_deriv(t, x, u, motor_params)

    # Compare the result with the expected value
    np.testing.assert_allclose(result, expected_result)

def test_dc_motor_deriv_current(motor_params):
    t = 0.0  # Time
    x = np.array([10.0, 0.0])  # Initial states [i_m, omega]
    u = np.array([0.0, 0.0])  # Inputs
    expected_result = np.array([ -200., 100.])  # Expected derivative of state vector

    result = dc_motor_deriv(t, x, u, motor_params)

    # Compare the result with the expected value
    np.testing.assert_allclose(result, expected_result)

def test_dc_motor_deriv_saturation(motor_params):
    t = 0.0  # Time
    x = np.array([0.0, 0.0])  # Initial states [i_m, omega]
    u = np.array([1.1, 0.0])  # Inputs
    expected_result = np.array([ 200., 0.])  # Expected derivative of state vector

    result = dc_motor_deriv(t, x, u, motor_params)

    # Compare the result with the expected value
    np.testing.assert_allclose(result, expected_result)
# Run tests
if __name__ == "__main__":
    pytest.main()
