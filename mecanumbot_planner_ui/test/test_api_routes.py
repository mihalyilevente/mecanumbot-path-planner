import pytest

try:
    import flask  # noqa: F401
except ImportError:  # pragma: no cover
    flask = None

pytestmark = pytest.mark.skipif(flask is None, reason='Flask is not installed')

from mecanumbot_planner_ui.app import create_app  # noqa: E402
from mecanumbot_planner_ui.planner_catalog import planner_list  # noqa: E402


class FakeBridge:
    def __init__(self):
        self._planner = 'Shortest'
        self._goal = None

    def status(self):
        return {
            'selected_planner': self._planner,
            'nav_status': 'idle',
            'last_error': '',
            'goal_source': 'free',
            'scenario': None,
            'pair': None,
            'map_ready': True,
            'pose_ready': True,
            'navigate_ready': True,
            'compute_ready': True,
            'pose_source': 'odom',
        }

    def state(self):
        return {
            'map': None,
            'robot': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'goal': self._goal,
            'scenario_start': None,
            'active_path': [],
            'compare_paths': {},
            'planners': planner_list(),
        }

    def scenarios(self):
        return ['demo']

    def scenario_detail(self, name):
        return {
            'ok': True,
            'map': name,
            'pairs': [{'name': 'a_to_b', 'start': [0, 0, 0], 'goal': [1, 0, 0]}],
        }

    def select_planner(self, planner_id):
        self._planner = planner_id
        return {'ok': True, 'planner_id': planner_id}

    def set_goal(self, x, y, yaw):
        self._goal = {'x': x, 'y': y, 'yaw': yaw}
        return {'ok': True, 'goal': self._goal}

    def set_goal_from_scenario(self, scenario, pair):
        self._goal = {'x': 1.0, 'y': 0.0, 'yaw': 0.0}
        return {'ok': True, 'goal': self._goal}

    def compare(self):
        return {'ok': True, 'paths': {}}

    def drive(self):
        return {'ok': True, 'planner_id': self._planner}

    def cancel(self):
        return {'ok': True}


@pytest.fixture
def client():
    app = create_app(FakeBridge())
    app.config['TESTING'] = True
    return app.test_client()


def test_index_and_planners(client):
    assert client.get('/').status_code == 200
    data = client.get('/api/planners').get_json()
    assert len(data['planners']) == 5


def test_select_goal_drive_cancel(client):
    assert client.post(
        '/api/select_planner', json={'planner_id': 'Safest'}
    ).get_json()['ok']
    assert client.post(
        '/api/goal', json={'x': 2.0, 'y': 3.0, 'yaw': 1.0}
    ).get_json()['ok']
    assert client.post('/api/drive').get_json()['ok']
    assert client.post('/api/cancel').get_json()['ok']
    assert client.post('/api/compare').get_json()['ok']


def test_scenarios(client):
    assert client.get('/api/scenarios').get_json()['scenarios'] == ['demo']
    detail = client.get('/api/scenarios/demo').get_json()
    assert detail['ok']
    assert detail['pairs'][0]['name'] == 'a_to_b'
