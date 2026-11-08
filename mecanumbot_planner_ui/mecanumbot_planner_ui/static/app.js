(() => {
  const canvas = document.getElementById('map-canvas');
  const ctx = canvas.getContext('2d');
  const plannerList = document.getElementById('planner-list');
  const scenarioSelect = document.getElementById('scenario-select');
  const pairSelect = document.getElementById('pair-select');
  const statusBox = document.getElementById('status-box');

  let planners = [];
  let selectedPlanner = 'Shortest';
  let latestState = null;
  let mapImage = null;
  let mapVersion = -1;
  let dragStart = null;

  async function getJson(url) {
    const res = await fetch(url);
    return res.json();
  }

  async function postJson(url, body) {
    const res = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {}),
    });
    return res.json();
  }

  function renderPlanners() {
    plannerList.innerHTML = '';
    planners.forEach((p) => {
      const label = document.createElement('label');
      label.className = p.id === selectedPlanner ? 'active' : '';
      label.innerHTML =
        `<input type="radio" name="planner" value="${p.id}">` +
        `<span class="swatch" style="background:${p.colour}"></span>` +
        `<span>${p.id}</span>`;
      const input = label.querySelector('input');
      input.checked = p.id === selectedPlanner;
      input.addEventListener('change', async () => {
        selectedPlanner = p.id;
        await postJson('/api/select_planner', { planner_id: p.id });
        renderPlanners();
      });
      plannerList.appendChild(label);
    });
  }

  async function loadScenarios() {
    const data = await getJson('/api/scenarios');
    scenarioSelect.innerHTML = '';
    (data.scenarios || []).forEach((name) => {
      const opt = document.createElement('option');
      opt.value = name;
      opt.textContent = name;
      scenarioSelect.appendChild(opt);
    });
    if (data.scenarios && data.scenarios.length) {
      await loadPairs(data.scenarios[0]);
    }
  }

  async function loadPairs(name) {
    const data = await getJson(`/api/scenarios/${encodeURIComponent(name)}`);
    pairSelect.innerHTML = '';
    if (!data.ok) {
      return;
    }
    (data.pairs || []).forEach((pair) => {
      const opt = document.createElement('option');
      opt.value = pair.name;
      opt.textContent = pair.name;
      pairSelect.appendChild(opt);
    });
  }

  scenarioSelect.addEventListener('change', () => loadPairs(scenarioSelect.value));

  document.getElementById('btn-load-scenario').addEventListener('click', async () => {
    const result = await postJson('/api/goal/scenario', {
      scenario: scenarioSelect.value,
      pair: pairSelect.value,
    });
    if (!result.ok) {
      alert(result.error || 'Failed to load scenario');
    }
  });

  document.getElementById('btn-compare').addEventListener('click', async () => {
    const result = await postJson('/api/compare', {});
    if (!result.ok) {
      alert(result.error || 'Compare failed');
    }
  });

  document.getElementById('btn-drive').addEventListener('click', async () => {
    const result = await postJson('/api/drive', {});
    if (!result.ok) {
      alert(result.error || 'Drive failed');
    }
  });

  document.getElementById('btn-cancel').addEventListener('click', async () => {
    const result = await postJson('/api/cancel', {});
    if (!result.ok) {
      alert(result.error || 'Cancel failed');
    }
  });

  function worldToCanvas(map, x, y) {
    const origin = map.origin;
    const col = (x - origin.x) / map.resolution;
    const rowFromBottom = (y - origin.y) / map.resolution;
    const row = map.height - rowFromBottom;
    const scale = Math.min(canvas.width / map.width, canvas.height / map.height);
    const ox = (canvas.width - map.width * scale) / 2;
    const oy = (canvas.height - map.height * scale) / 2;
    return { x: ox + col * scale, y: oy + row * scale, scale };
  }

  function canvasToWorld(map, cx, cy) {
    const scale = Math.min(canvas.width / map.width, canvas.height / map.height);
    const ox = (canvas.width - map.width * scale) / 2;
    const oy = (canvas.height - map.height * scale) / 2;
    const col = (cx - ox) / scale;
    const row = (cy - oy) / scale;
    const rowFromBottom = map.height - row;
    const x = map.origin.x + col * map.resolution;
    const y = map.origin.y + rowFromBottom * map.resolution;
    return { x, y };
  }

  function drawPoseMarker(map, pose, colour, lengthPx) {
    if (!pose) return;
    const p = worldToCanvas(map, pose.x, pose.y);
    const len = lengthPx || 18;
    ctx.strokeStyle = colour;
    ctx.fillStyle = colour;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.beginPath();
    ctx.moveTo(p.x, p.y);
    ctx.lineTo(p.x + Math.cos(pose.yaw) * len, p.y - Math.sin(pose.yaw) * len);
    ctx.stroke();
  }

  function drawPath(map, points, colour, width) {
    if (!points || points.length < 2) return;
    ctx.strokeStyle = colour;
    ctx.lineWidth = width;
    ctx.beginPath();
    points.forEach((pt, i) => {
      const p = worldToCanvas(map, pt.x, pt.y);
      if (i === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    });
    ctx.stroke();
  }

  function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    const state = latestState;
    if (!state || !state.map) {
      ctx.fillStyle = '#9aabbc';
      ctx.font = '16px sans-serif';
      ctx.fillText('Waiting for /map …', 24, 40);
      return;
    }
    const map = state.map;
    const scale = Math.min(canvas.width / map.width, canvas.height / map.height);
    const ox = (canvas.width - map.width * scale) / 2;
    const oy = (canvas.height - map.height * scale) / 2;

    if (mapImage) {
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(mapImage, ox, oy, map.width * scale, map.height * scale);
    }

    const colourOf = {};
    (state.planners || planners).forEach((p) => {
      colourOf[p.id] = p.colour;
    });

    Object.keys(state.compare_paths || {}).forEach((id) => {
      drawPath(map, state.compare_paths[id], colourOf[id] || '#ccc', 2);
    });
    drawPath(map, state.active_path, colourOf[selectedPlanner] || '#5ecfff', 3);

    drawPoseMarker(map, state.scenario_start, '#aaaaaa', 14);
    drawPoseMarker(map, state.goal, '#ff5555', 22);
    drawPoseMarker(map, state.robot, '#2ec4b6', 20);
  }

  function maybeUpdateMapImage(map) {
    if (!map || !map.png_base64) return;
    if (map.version === mapVersion && mapImage) return;
    mapVersion = map.version;
    const img = new Image();
    img.onload = () => {
      mapImage = img;
      draw();
    };
    img.src = `data:image/png;base64,${map.png_base64}`;
  }

  function fitCanvas() {
    const wrap = canvas.parentElement;
    const maxW = Math.max(320, wrap.clientWidth - 16);
    const maxH = Math.max(240, window.innerHeight - 120);
    canvas.width = Math.floor(maxW);
    canvas.height = Math.floor(Math.min(maxH, maxW * 0.75));
    draw();
  }

  canvas.addEventListener('pointerdown', (ev) => {
    if (!latestState || !latestState.map) return;
    const rect = canvas.getBoundingClientRect();
    const cx = ((ev.clientX - rect.left) / rect.width) * canvas.width;
    const cy = ((ev.clientY - rect.top) / rect.height) * canvas.height;
    const world = canvasToWorld(latestState.map, cx, cy);
    dragStart = { x: world.x, y: world.y, cx, cy };
    canvas.setPointerCapture(ev.pointerId);
  });

  canvas.addEventListener('pointerup', async (ev) => {
    if (!dragStart || !latestState || !latestState.map) return;
    const rect = canvas.getBoundingClientRect();
    const cx = ((ev.clientX - rect.left) / rect.width) * canvas.width;
    const cy = ((ev.clientY - rect.top) / rect.height) * canvas.height;
    const end = canvasToWorld(latestState.map, cx, cy);
    let yaw = 0;
    const dx = end.x - dragStart.x;
    const dy = end.y - dragStart.y;
    if (Math.hypot(dx, dy) > 0.05) {
      yaw = Math.atan2(dy, dx);
    }
    // Press point is the goal; drag only sets heading (same as RViz 2D Goal Pose).
    const goalX = dragStart.x;
    const goalY = dragStart.y;
    dragStart = null;
    await postJson('/api/goal', { x: goalX, y: goalY, yaw });
  });

  async function pollState() {
    try {
      latestState = await getJson('/api/state');
      if (latestState.selected_planner) {
        // keep local selection in sync if server changed
      }
      maybeUpdateMapImage(latestState.map);
      draw();
    } catch (err) {
      // leave last frame
    }
  }

  async function pollStatus() {
    try {
      const status = await getJson('/api/status');
      selectedPlanner = status.selected_planner || selectedPlanner;
      renderPlanners();
      statusBox.textContent = [
        `planner: ${status.selected_planner}`,
        `nav: ${status.nav_status}`,
        `map: ${status.map_ready ? 'ok' : 'waiting'}`,
        `pose: ${status.pose_ready ? 'ok' : 'waiting'} (${status.pose_source})`,
        `navigate: ${status.navigate_ready ? 'ready' : 'down'}`,
        `compute: ${status.compute_ready ? 'ready' : 'down'}`,
        status.scenario ? `scenario: ${status.scenario} / ${status.pair}` : 'scenario: (none)',
        status.last_error ? `error: ${status.last_error}` : '',
      ]
        .filter(Boolean)
        .join('\n');
    } catch (err) {
      statusBox.textContent = 'Lost connection to UI node';
    }
  }

  async function boot() {
    const data = await getJson('/api/planners');
    planners = data.planners || [];
    renderPlanners();
    await loadScenarios();
    fitCanvas();
    window.addEventListener('resize', fitCanvas);
    setInterval(pollState, 200);
    setInterval(pollStatus, 500);
    pollState();
    pollStatus();
  }

  boot();
})();
