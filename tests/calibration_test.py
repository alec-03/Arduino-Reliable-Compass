#!/usr/bin/env python3
"""Check calibration fitting and the public C++ assignment output without hardware."""
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
import runpy
import subprocess
import tempfile
import numpy as np

library = Path(__file__).resolve().parents[1]
tool = runpy.run_path(str(library / 'tools/calibrate.py'))
rng = np.random.default_rng(42)
vectors = rng.normal(size=(200, 3))
vectors /= np.linalg.norm(vectors, axis=1)[:, None]
center = np.array([3.0, -4.0, 5.0])
fit = tool['sphere_fit'](center + 12 * vectors, 'synthetic')
np.testing.assert_allclose(fit, [3, -4, 5, 12], atol=1e-6)
bias = tool['gyro_bias'](np.tile([0.1, -0.2, 0.3], (20, 1)))
np.testing.assert_allclose(bias, [0.1, -0.2, 0.3])
output = StringIO()
with redirect_stdout(output):
    tool['print_constants'](fit, (0.01, -0.02, 0.03, 1.02), bias)
assignments = '\n'.join(line for line in output.getvalue().splitlines()
                        if line.startswith(('quaternion_compass::', 'calibration.')))
with tempfile.TemporaryDirectory() as directory:
    cpp = Path(directory) / 'calibration.cpp'
    cpp.write_text('#include <QuaternionCompass.h>\nvoid configure() {\n' + assignments + '\n}\n')
    subprocess.run(['c++', '-std=c++14', '-Wall', '-Wextra', '-Werror', '-fsyntax-only',
                    '-I' + str(library / 'src'), str(cpp)], check=True)
print('Calibration fit, gyro bias, and generated C++ assignments passed')
