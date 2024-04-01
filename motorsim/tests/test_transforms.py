import numpy as np
import pytest
from motorsim.utils.clark_park_transforms import clarke_park

def test_clarke_park():
    # Define input parameters
    theta_e = np.pi / 4  # Electrical angle (45 degrees)
    u_phase = np.array([1, 2, 3])  # Stator voltages (Va=1V, Vb=2V, Vc=3V)

    # Expected output (manually calculated)
    expected_output = np.array([1.5, 0.5])  # Expected rotor voltages (Vd=1.5V, Vq=0.5V)

    # Call the function
    result = clarke_park(theta_e, u_phase)

    # Check if the result matches the expected output
    np.testing.assert_allclose(result, expected_output)

# Run the test
if __name__ == "__main__":
    pytest.main()