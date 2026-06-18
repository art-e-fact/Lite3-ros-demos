import pytest

try:
    from artefacts_toolkit.config import get_artefacts_params
except Exception:
    get_artefacts_params = None


def _parse_bool(value) -> bool:
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def _artefacts_headless() -> bool:
    if get_artefacts_params is None:
        return False
    try:
        params = get_artefacts_params()
    except Exception:
        return False
    return _parse_bool(params.get('headless', 'false'))


def pytest_addoption(parser):
    parser.addoption(
        '--headless',
        action='store_true',
        default=False,
        help='Run simulation without GUI (no MuJoCo viewer, no Rerun spawn)',
    )


@pytest.fixture(scope='session')
def headless(request):
    if request.config.getoption('--headless'):
        return True
    return _artefacts_headless()
