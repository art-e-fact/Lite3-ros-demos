import pytest

from robot_profiles import get_robot_profile

try:
    from artefacts_toolkit.config import get_artefacts_params
except Exception:
    get_artefacts_params = None


def _parse_bool(value) -> bool:
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def _artefacts_params() -> dict:
    if get_artefacts_params is None:
        return {}
    try:
        return get_artefacts_params()
    except Exception:
        return {}


def _artefacts_headless() -> bool:
    return _parse_bool(_artefacts_params().get('headless', 'false'))


def pytest_addoption(parser):
    parser.addoption(
        '--headless',
        action='store_true',
        default=False,
        help='Run simulation without GUI (no MuJoCo viewer, no Rerun spawn)',
    )
    parser.addoption(
        '--robot',
        action='store',
        default='lite3',
        choices=['lite3', 'm20'],
        help='Robot profile for integration tests (lite3 or m20)',
    )
    parser.addoption(
        '--simulator',
        action='store',
        default='newton',
        choices=['newton', 'mujoco'],
        help='Simulator backend for integration tests (newton or mujoco)',
    )


@pytest.fixture(scope='session')
def headless(request):
    if request.config.getoption('--headless'):
        return True
    return _artefacts_headless()


@pytest.fixture(scope='session')
def robot_profile(request):
    params = _artefacts_params()
    robot_name = params.get('robot') or request.config.getoption('--robot')
    return get_robot_profile(str(robot_name).strip().lower())


@pytest.fixture(scope='session')
def simulator(request):
    return str(request.config.getoption('--simulator')).strip().lower()
