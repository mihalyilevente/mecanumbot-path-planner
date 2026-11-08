"""Flask routes for the planner UI."""

from __future__ import annotations

import os
from typing import Any

from mecanumbot_planner_ui.planner_catalog import planner_list

try:
    from flask import Flask, jsonify, render_template, request
except ImportError:  # pragma: no cover
    Flask = None  # type: ignore[misc, assignment]


def _package_dir() -> str:
    return os.path.dirname(os.path.abspath(__file__))


def create_app(bridge: Any) -> Any:
    if Flask is None:
        raise RuntimeError('Flask is not installed; apt install python3-flask')

    root = _package_dir()
    app = Flask(
        __name__,
        template_folder=os.path.join(root, 'templates'),
        static_folder=os.path.join(root, 'static'),
        static_url_path='/static',
    )

    @app.get('/')
    def index():
        return render_template('index.html')

    @app.get('/api/status')
    def api_status():
        return jsonify(bridge.status())

    @app.get('/api/state')
    def api_state():
        return jsonify(bridge.state())

    @app.get('/api/planners')
    def api_planners():
        return jsonify({'planners': planner_list()})

    @app.get('/api/scenarios')
    def api_scenarios():
        return jsonify({'scenarios': bridge.scenarios()})

    @app.get('/api/scenarios/<name>')
    def api_scenario_detail(name: str):
        return jsonify(bridge.scenario_detail(name))

    @app.post('/api/select_planner')
    def api_select_planner():
        body = request.get_json(silent=True) or {}
        planner_id = body.get('planner_id')
        if not planner_id:
            return jsonify({'ok': False, 'error': 'planner_id required'}), 400
        return jsonify(bridge.select_planner(planner_id))

    @app.post('/api/goal')
    def api_goal():
        body = request.get_json(silent=True) or {}
        try:
            x = float(body['x'])
            y = float(body['y'])
            yaw = float(body.get('yaw', 0.0))
        except (KeyError, TypeError, ValueError):
            return jsonify({'ok': False, 'error': 'x, y, yaw required'}), 400
        return jsonify(bridge.set_goal(x, y, yaw))

    @app.post('/api/goal/scenario')
    def api_goal_scenario():
        body = request.get_json(silent=True) or {}
        scenario = body.get('scenario')
        pair = body.get('pair')
        if not scenario or not pair:
            return jsonify({'ok': False, 'error': 'scenario and pair required'}), 400
        result = bridge.set_goal_from_scenario(scenario, pair)
        code = 200 if result.get('ok') else 400
        return jsonify(result), code

    @app.post('/api/compare')
    def api_compare():
        return jsonify(bridge.compare())

    @app.post('/api/drive')
    def api_drive():
        return jsonify(bridge.drive())

    @app.post('/api/cancel')
    def api_cancel():
        return jsonify(bridge.cancel())

    return app
