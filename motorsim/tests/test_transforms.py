import numpy as np
import pytest
from motorsim.utils.clark_park_transforms import clarke_park, clarke_park_inv

def test_clarke_park():
    # Define input parameters
    theta_e = np.pi / 4  # Electrical angle (45 degrees)
    
    u_phase = np.array([
        np.sin(theta_e), 
        np.sin(theta_e - np.deg2rad(120)), 
        np.sin(theta_e - np.deg2rad(2*120))
        ])  # Stator voltages (Va, Vb, Vc)
    # Expected output (manually calculated)
    expected_output = np.array([0,  -1.])  # Expected rotor frame voltages

    # Call the function
    result = clarke_park(theta_e, u_phase)
    print("clarke_park", result)
    # Check if the result matches the expected output
    np.testing.assert_allclose(result, expected_output, atol=1e-12)

def test_inv_clarke_park():
    # Define input parameters
    theta_e = np.pi / 4  # Electrical angle (45 degrees)
    u_qd = np.array([0,  -1])  # Stator voltages (Va=1V, Vb=2V, Vc=3V)

    # Expected output (manually calculated)
    expected_output  = np.array([
        np.sin(theta_e), 
        np.sin(theta_e - np.deg2rad(120)), 
        np.sin(theta_e - np.deg2rad(2*120))
        ])  # Stator voltages (Va, Vb, Vc)

    # Call the function
    result = clarke_park_inv(theta_e, u_qd)
    print("clarke_park_inv", result)
    # Check if the result matches the expected output
    np.testing.assert_allclose(result, expected_output)

# Run the test
if __name__ == "__main__":
    pytest.main()
