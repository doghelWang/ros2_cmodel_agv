#!/usr/bin/env python3
"""
Updates generate_console.py with enhanced 2D view features:
1. Topology lines & nodes (e.p1 -> e.p2) with rails and direction
2. Environment contours (outer warehouse walls, shelf blocks with names, injected obstacles)
3. Laser contours (closed laser FOV sector polygon with soft tint, perimeter contour line with color zones, hit points)
4. Planned path information (flowing animated dashed line, waypoint numbers, target beacon ripple, floating path info card)
5. Interactive 2D layer switcher toolbar (Topology, Environment, Laser, Path, Stations, Reset/Fit)
"""

with open("generate_console.py", "r", encoding="utf-8") as f:
    code = f.read()

# 1. HTML Layer Bar & Path Info Card
old_canvas_html = """      <!-- 2D Canvas (SLAM 栅格、激光雷达命中线、车身底盘、路径) -->
      <canvas id="canvas-2d" class="w-full h-full block cursor-grab bg-slate-50"></canvas>

      <!-- 3D Three.js Canvas (立体车身、货架障碍物、3D 动态激光雷达点云) -->
      <canvas id="canvas-3d" class="w-full h-full block hidden bg-slate-100"></canvas>

      <!-- 悬浮遥测 HUD (左上角) -->"""

new_canvas_html = """      <!-- 2D Canvas (SLAM 栅格、激光雷达命中线、车身底盘、路径) -->
      <canvas id="canvas-2d" class="w-full h-full block cursor-grab bg-slate-50"></canvas>

      <!-- 3D Three.js Canvas (立体车身、货架障碍物、3D 动态激光雷达点云) -->
      <canvas id="canvas-3d" class="w-full h-full block hidden bg-slate-100"></canvas>

      <!-- 2D 视图图层与信息显示控制器 (右上角悬浮胶囊栏) -->
      <div id="canvas-2d-toolbar" class="absolute top-3 right-3 flex items-center gap-1 bg-white/95 backdrop-blur border border-slate-200 rounded-lg p-1 shadow-md z-10 text-[11px]">
        <button id="layer-btn-topo" onclick="toggleLayer('topology')" class="px-2 py-0.5 rounded font-medium bg-blue-50 text-blue-700 border border-blue-200 flex items-center gap-1 transition-all shadow-xs" title="显示/隐藏 拓扑导轨路网">
          <i data-lucide="git-commit" class="w-3 h-3"></i> 拓扑线
        </button>
        <button id="layer-btn-env" onclick="toggleLayer('environment')" class="px-2 py-0.5 rounded font-medium bg-blue-50 text-blue-700 border border-blue-200 flex items-center gap-1 transition-all shadow-xs" title="显示/隐藏 仓库墙体与货架轮廓">
          <i data-lucide="warehouse" class="w-3 h-3"></i> 环境轮廓
        </button>
        <button id="layer-btn-laser" onclick="toggleLayer('laser')" class="px-2 py-0.5 rounded font-medium bg-blue-50 text-blue-700 border border-blue-200 flex items-center gap-1 transition-all shadow-xs" title="显示/隐藏 激光雷达扫描视场与轮廓">
          <i data-lucide="radar" class="w-3 h-3"></i> 激光轮廓
        </button>
        <button id="layer-btn-path" onclick="toggleLayer('path')" class="px-2 py-0.5 rounded font-medium bg-blue-50 text-blue-700 border border-blue-200 flex items-center gap-1 transition-all shadow-xs" title="显示/隐藏 实时规划导航路径">
          <i data-lucide="route" class="w-3 h-3"></i> 规划路径
        </button>
        <button id="layer-btn-stations" onclick="toggleLayer('stations')" class="px-2 py-0.5 rounded font-medium bg-blue-50 text-blue-700 border border-blue-200 flex items-center gap-1 transition-all shadow-xs" title="显示/隐藏 仓储工位与对齐标">
          <i data-lucide="map-pin" class="w-3 h-3"></i> 工位标注
        </button>
        <div class="h-3 w-[1px] bg-slate-200 mx-0.5"></div>
        <button onclick="reset2DView()" title="视角自适应居中" class="p-1 rounded hover:bg-slate-100 text-slate-600 transition-colors">
          <i data-lucide="maximize" class="w-3.5 h-3.5"></i>
        </button>
      </div>

      <!-- 规划路径信息卡片 (右下角悬浮，有活动路径时展示) -->
      <div id="path-info-card" class="absolute bottom-3 right-3 bg-white/95 backdrop-blur border border-blue-200 rounded-xl p-2.5 shadow-md z-10 hidden min-w-[250px] text-xs transition-all">
        <div class="flex items-center justify-between pb-1.5 border-b border-slate-100 mb-1.5">
          <span class="font-bold text-blue-800 flex items-center gap-1">
            <i data-lucide="route" class="w-3.5 h-3.5 text-blue-600"></i>
            实时路径规划与导引信息
          </span>
          <span id="path-card-status" class="px-1.5 py-0.2 rounded text-[10px] font-bold bg-blue-100 text-blue-800 font-mono">
            ● 执行中
          </span>
        </div>
        <div class="space-y-1 text-[11px] text-slate-600 font-mono-num">
          <div class="flex justify-between"><span>规划引擎:</span><span id="path-card-engine" class="font-bold text-slate-800">Dijkstra 拓扑路网</span></div>
          <div class="flex justify-between"><span>规划航点总数:</span><span id="path-card-waypoints" class="font-bold text-slate-800">0 个航点</span></div>
          <div class="flex justify-between"><span>剩余里程估算:</span><span id="path-card-dist" class="font-bold text-blue-700">0.00 m</span></div>
          <div class="flex justify-between"><span>目标工位坐标:</span><span id="path-card-goal" class="font-bold text-slate-800 truncate max-w-[140px]">(0.0, 0.0)</span></div>
        </div>
      </div>

      <!-- 悬浮遥测 HUD (左上角) -->"""

code = code.replace(old_canvas_html, new_canvas_html)

# 2. Add layerConfig and toggle functions in JavaScript
old_js_globals = """    // 2D 视口变换参数
    const view2D = {
      panX: 0,
      panY: 0,
      scale: 35.0, // pixels per meter
      isDragging: false,
      dragStartX: 0,
      dragStartY: 0
    };"""

new_js_globals = """    // 2D 视口变换参数
    const view2D = {
      panX: 0,
      panY: 0,
      scale: 35.0, // pixels per meter
      isDragging: false,
      dragStartX: 0,
      dragStartY: 0
    };

    // 2D 视图图层可见性控制 (满足用户自定义查看拓扑线、环境轮廓、激光轮廓、规划路径)
    const layerConfig = {
      topology: true,
      environment: true,
      laser: true,
      path: true,
      stations: true
    };

    function toggleLayer(layerName) {
      if (layerConfig[layerName] !== undefined) {
        layerConfig[layerName] = !layerConfig[layerName];
        const btn = document.getElementById('layer-btn-' + (layerName === 'topology' ? 'topo' : layerName === 'environment' ? 'env' : layerName));
        if (btn) {
          if (layerConfig[layerName]) {
            btn.className = 'px-2 py-0.5 rounded font-medium bg-blue-50 text-blue-700 border border-blue-200 flex items-center gap-1 transition-all shadow-xs';
          } else {
            btn.className = 'px-2 py-0.5 rounded font-medium bg-white text-slate-400 border border-slate-200 flex items-center gap-1 transition-all shadow-xs';
          }
        }
      }
    }

    function reset2DView() {
      if (!cv2D) return;
      view2D.panX = cv2D.width / 2;
      view2D.panY = cv2D.height / 2;
      view2D.scale = 35.0 * window.devicePixelRatio;
    }"""

code = code.replace(old_js_globals, new_js_globals)

# 3. Replace draw2DScene() with the comprehensive multi-layer renderer
old_draw2d = """    function draw2DScene() {
      const W = cv2D.width, H = cv2D.height;
      ctx2D.clearRect(0, 0, W, H);

      const toScreen = (wx, wy) => ({
        sx: view2D.panX + wx * view2D.scale,
        sy: view2D.panY - wy * view2D.scale
      });

      // 1. 背景细栅格 (1米间隔)
      ctx2D.strokeStyle = '#e2e8f0';
      ctx2D.lineWidth = 1;
      const gridSpan = 15;
      for (let i = -gridSpan; i <= gridSpan; i++) {
        const p1 = toScreen(i, -gridSpan);
        const p2 = toScreen(i, gridSpan);
        ctx2D.beginPath();
        ctx2D.moveTo(p1.sx, p1.sy);
        ctx2D.lineTo(p2.sx, p2.sy);
        ctx2D.stroke();

        const p3 = toScreen(-gridSpan, i);
        const p4 = toScreen(gridSpan, i);
        ctx2D.beginPath();
        ctx2D.moveTo(p3.sx, p3.sy);
        ctx2D.lineTo(p4.sx, p4.sy);
        ctx2D.stroke();
      }

      // 2. 拓扑路网 (Topology Graph)
      if (telemetry && telemetry.topo_graph) {
        ctx2D.strokeStyle = '#93c5fd';
        ctx2D.lineWidth = 2 * window.devicePixelRatio;
        const nodes = telemetry.topo_graph.nodes || {};
        const edges = telemetry.topo_graph.edges || [];
        edges.forEach(e => {
          const n1 = nodes[e[0]], n2 = nodes[e[1]];
          if (n1 && n2) {
            const s1 = toScreen(n1.x, n1.y);
            const s2 = toScreen(n2.x, n2.y);
            ctx2D.beginPath();
            ctx2D.moveTo(s1.sx, s1.sy);
            ctx2D.lineTo(s2.sx, s2.sy);
            ctx2D.stroke();
          }
        });

        // 工位节点图标
        Object.entries(nodes).forEach(([k, n]) => {
          const s = toScreen(n.x, n.y);
          ctx2D.fillStyle = '#3b82f6';
          ctx2D.beginPath();
          ctx2D.arc(s.sx, s.sy, 6 * window.devicePixelRatio, 0, Math.PI * 2);
          ctx2D.fill();
          ctx2D.fillStyle = '#1e293b';
          ctx2D.font = `${10 * window.devicePixelRatio}px sans-serif`;
          ctx2D.fillText(k, s.sx + 8 * window.devicePixelRatio, s.sy + 4 * window.devicePixelRatio);
        });
      }

      // 3. 规划轨迹动线 (Navigation Path)
      if (telemetry && telemetry.nav_path && telemetry.nav_path.length > 1) {
        ctx2D.strokeStyle = '#2563eb';
        ctx2D.lineWidth = 3 * window.devicePixelRatio;
        ctx2D.setLineDash([6 * window.devicePixelRatio, 4 * window.devicePixelRatio]);
        ctx2D.beginPath();
        telemetry.nav_path.forEach((pt, idx) => {
          const s = toScreen(pt.x, pt.y);
          if (idx === 0) ctx2D.moveTo(s.sx, s.sy);
          else ctx2D.lineTo(s.sx, s.sy);
        });
        ctx2D.stroke();
        ctx2D.setLineDash([]);
      }

      // 4. 障碍物渲染
      if (injectedObstacles && injectedObstacles.length > 0) {
        ctx2D.fillStyle = '#fde68a';
        ctx2D.strokeStyle = '#d97706';
        ctx2D.lineWidth = 2 * window.devicePixelRatio;
        injectedObstacles.forEach(obs => {
          const s = toScreen(obs.x - obs.w / 2, obs.y + obs.h / 2);
          const w = obs.w * view2D.scale;
          const h = obs.h * view2D.scale;
          ctx2D.fillRect(s.sx, s.sy, w, h);
          ctx2D.strokeRect(s.sx, s.sy, w, h);
        });
      }

      // 5. 激光雷达扫描射线与命中点
      if (telemetry && telemetry.laser_scan && telemetry.laser_scan.ranges) {
        const ranges = telemetry.laser_scan.ranges;
        const angleMin = telemetry.laser_scan.angle_min || -Math.PI;
        const angleInc = telemetry.laser_scan.angle_increment || (Math.PI * 2 / ranges.length);
        const robotS = toScreen(deadReckonPose.x, deadReckonPose.y);

        ctx2D.fillStyle = '#10b981';
        ranges.forEach((r, idx) => {
          if (r > 0.05 && r < 12.0) {
            const beamAngle = deadReckonPose.yaw + angleMin + idx * angleInc;
            const hitX = deadReckonPose.x + Math.cos(beamAngle) * r;
            const hitY = deadReckonPose.y + Math.sin(beamAngle) * r;
            const hitS = toScreen(hitX, hitY);
            ctx2D.beginPath();
            ctx2D.arc(hitS.sx, hitS.sy, 2 * window.devicePixelRatio, 0, Math.PI * 2);
            ctx2D.fill();
          }
        });
      }

      // 6. AMR 机器人车身底盘
      const rS = toScreen(deadReckonPose.x, deadReckonPose.y);
      ctx2D.save();
      ctx2D.translate(rS.sx, rS.sy);
      ctx2D.rotate(-deadReckonPose.yaw); // Canvas Y 轴向下，旋转取反

      const carW = 0.8 * view2D.scale;
      const carL = 1.2 * view2D.scale;

      // 车身矩形
      ctx2D.fillStyle = '#2563eb';
      ctx2D.fillRect(-carL / 2, -carW / 2, carL, carW);
      ctx2D.strokeStyle = '#1e3a8a';
      ctx2D.lineWidth = 2 * window.devicePixelRatio;
      ctx2D.strokeRect(-carL / 2, -carW / 2, carL, carW);

      // 车头方向标 (红三角)
      ctx2D.fillStyle = '#ef4444';
      ctx2D.beginPath();
      ctx2D.moveTo(carL / 2 + 10 * window.devicePixelRatio, 0);
      ctx2D.lineTo(carL / 2 - 4 * window.devicePixelRatio, -8 * window.devicePixelRatio);
      ctx2D.lineTo(carL / 2 - 4 * window.devicePixelRatio, 8 * window.devicePixelRatio);
      ctx2D.closePath();
      ctx2D.fill();

      // 激光雷达传感器原点 (绿圆)
      ctx2D.fillStyle = '#10b981';
      ctx2D.beginPath();
      ctx2D.arc(0, 0, 5 * window.devicePixelRatio, 0, Math.PI * 2);
      ctx2D.fill();

      ctx2D.restore();
    }"""

new_draw2d = """    function draw2DScene() {
      const W = cv2D.width, H = cv2D.height;
      ctx2D.clearRect(0, 0, W, H);

      const dpr = window.devicePixelRatio || 1;
      const toScreen = (wx, wy) => ({
        sx: view2D.panX + wx * view2D.scale,
        sy: view2D.panY - wy * view2D.scale
      });

      // ------------------------------------------------------------
      // 1. 细网格标尺与坐标基准 (1米主栅格 + 0.5米辅助格)
      // ------------------------------------------------------------
      ctx2D.strokeStyle = '#f1f5f9';
      ctx2D.lineWidth = 1;
      const gridSpan = 15;
      for (let i = -gridSpan; i <= gridSpan; i += 0.5) {
        const p1 = toScreen(i, -gridSpan);
        const p2 = toScreen(i, gridSpan);
        ctx2D.beginPath();
        ctx2D.moveTo(p1.sx, p1.sy);
        ctx2D.lineTo(p2.sx, p2.sy);
        ctx2D.stroke();

        const p3 = toScreen(-gridSpan, i);
        const p4 = toScreen(gridSpan, i);
        ctx2D.beginPath();
        ctx2D.moveTo(p3.sx, p3.sy);
        ctx2D.lineTo(p4.sx, p4.sy);
        ctx2D.stroke();
      }

      ctx2D.strokeStyle = '#e2e8f0';
      ctx2D.lineWidth = 1.2 * dpr;
      for (let i = -gridSpan; i <= gridSpan; i++) {
        const p1 = toScreen(i, -gridSpan);
        const p2 = toScreen(i, gridSpan);
        ctx2D.beginPath();
        ctx2D.moveTo(p1.sx, p1.sy);
        ctx2D.lineTo(p2.sx, p2.sy);
        ctx2D.stroke();

        const p3 = toScreen(-gridSpan, i);
        const p4 = toScreen(gridSpan, i);
        ctx2D.beginPath();
        ctx2D.moveTo(p3.sx, p3.sy);
        ctx2D.lineTo(p4.sx, p4.sy);
        ctx2D.stroke();
      }

      // ------------------------------------------------------------
      // 2. 环境轮廓 (ENVIRONMENT CONTOURS: 墙体与立体货架)
      // ------------------------------------------------------------
      if (layerConfig.environment && telemetry && telemetry.scenario_metadata) {
        const meta = telemetry.scenario_metadata;

        // 2.1 外部库区围墙 (Perimeter Walls)
        if (meta.walls && Array.isArray(meta.walls)) {
          ctx2D.strokeStyle = '#334155';
          ctx2D.lineWidth = 4 * dpr;
          ctx2D.lineCap = 'round';
          meta.walls.forEach(w => {
            if (Array.isArray(w) && w.length >= 4) {
              const s1 = toScreen(w[0], w[1]);
              const s2 = toScreen(w[2], w[3]);
              ctx2D.beginPath();
              ctx2D.moveTo(s1.sx, s1.sy);
              ctx2D.lineTo(s2.sx, s2.sy);
              ctx2D.stroke();
            }
          });
        }

        // 2.2 仓储立体货架排架 (Shelves Racks)
        if (meta.shelves && Array.isArray(meta.shelves)) {
          meta.shelves.forEach(sh => {
            const pMin = toScreen(Math.min(sh.x1, sh.x2), Math.max(sh.y1, sh.y2));
            const pMax = toScreen(Math.max(sh.x1, sh.x2), Math.min(sh.y1, sh.y2));
            const rw = pMax.sx - pMin.sx;
            const rh = pMax.sy - pMin.sy;

            // 货架实体阴影与基底
            ctx2D.fillStyle = '#1e293b';
            ctx2D.fillRect(pMin.sx, pMin.sy, rw, rh);

            // 货架发光边框
            ctx2D.strokeStyle = sh.border || '#38bdf8';
            ctx2D.lineWidth = 2 * dpr;
            ctx2D.strokeRect(pMin.sx, pMin.sy, rw, rh);

            // 货架内部隔板细线
            ctx2D.strokeStyle = 'rgba(255, 255, 255, 0.15)';
            ctx2D.lineWidth = 1;
            ctx2D.beginPath();
            ctx2D.moveTo(pMin.sx, pMin.sy + rh / 2);
            ctx2D.lineTo(pMin.sx + rw, pMin.sy + rh / 2);
            ctx2D.moveTo(pMin.sx + rw / 2, pMin.sy);
            ctx2D.lineTo(pMin.sx + rw / 2, pMin.sy + rh);
            ctx2D.stroke();

            // 货架名称标签
            if (sh.name) {
              ctx2D.fillStyle = '#f8fafc';
              ctx2D.font = `bold ${Math.max(9, Math.min(12, 11 * dpr))}px sans-serif`;
              ctx2D.textAlign = 'center';
              ctx2D.textBaseline = 'middle';
              ctx2D.fillText(sh.name, pMin.sx + rw / 2, pMin.sy + rh / 2);
            }
          });
        }

        // 2.3 动态扰动障碍物 (Injected Dynamic Obstacles)
        if (injectedObstacles && injectedObstacles.length > 0) {
          injectedObstacles.forEach(obs => {
            const s = toScreen(obs.x - obs.w / 2, obs.y + obs.h / 2);
            const w = obs.w * view2D.scale;
            const h = obs.h * view2D.scale;

            const colorMap = {
              pallet: { fill: '#fef3c7', stroke: '#d97706', text: '🪵 木栈板' },
              shelf: { fill: '#ede9fe', stroke: '#7c3aed', text: '🏗️ 货架' },
              box: { fill: '#e0f2fe', stroke: '#0284c7', text: '📦 周转箱' },
              person: { fill: '#ffe4e6', stroke: '#e11d48', text: '👷 作业人员' }
            };
            const theme = colorMap[obs.type] || { fill: '#fde68a', stroke: '#d97706', text: '⚠️ 障碍物' };

            ctx2D.fillStyle = theme.fill;
            ctx2D.fillRect(s.sx, s.sy, w, h);
            ctx2D.strokeStyle = theme.stroke;
            ctx2D.lineWidth = 2 * dpr;
            ctx2D.strokeRect(s.sx, s.sy, w, h);

            ctx2D.fillStyle = '#1e293b';
            ctx2D.font = `bold ${10 * dpr}px sans-serif`;
            ctx2D.textAlign = 'center';
            ctx2D.textBaseline = 'middle';
            ctx2D.fillText(theme.text, s.sx + w / 2, s.sy + h / 2);
          });
        }
      }

      // ------------------------------------------------------------
      // 3. 拓扑路网 (TOPOLOGY LINES: 导轨动线与拓扑节点)
      // ------------------------------------------------------------
      if (layerConfig.topology && telemetry && telemetry.topo_graph) {
        const edges = telemetry.topo_graph.edges || [];
        const nodes = telemetry.topo_graph.nodes || {};

        // 3.1 拓扑导轨连接线 (带外发光与双轨效果)
        edges.forEach(e => {
          let p1 = null, p2 = null;
          if (e.p1 && e.p2) {
            p1 = e.p1; p2 = e.p2;
          } else if (e.from && e.to && nodes[e.from] && nodes[e.to]) {
            p1 = nodes[e.from]; p2 = nodes[e.to];
          }
          if (p1 && p2) {
            const s1 = toScreen(p1.x, p1.y);
            const s2 = toScreen(p2.x, p2.y);

            // 导轨外围柔和光晕
            ctx2D.strokeStyle = 'rgba(96, 165, 250, 0.28)';
            ctx2D.lineWidth = 6 * dpr;
            ctx2D.lineCap = 'round';
            ctx2D.beginPath();
            ctx2D.moveTo(s1.sx, s1.sy);
            ctx2D.lineTo(s2.sx, s2.sy);
            ctx2D.stroke();

            // 导轨核心导向实线
            ctx2D.strokeStyle = '#3b82f6';
            ctx2D.lineWidth = 2 * dpr;
            ctx2D.beginPath();
            ctx2D.moveTo(s1.sx, s1.sy);
            ctx2D.lineTo(s2.sx, s2.sy);
            ctx2D.stroke();
          }
        });

        // 3.2 拓扑节点 (Topological Node Points)
        Object.entries(nodes).forEach(([k, n]) => {
          const s = toScreen(n.x, n.y);
          // 外环
          ctx2D.fillStyle = '#ffffff';
          ctx2D.beginPath();
          ctx2D.arc(s.sx, s.sy, 5 * dpr, 0, Math.PI * 2);
          ctx2D.fill();
          ctx2D.strokeStyle = '#2563eb';
          ctx2D.lineWidth = 2 * dpr;
          ctx2D.stroke();
          // 内点
          ctx2D.fillStyle = '#2563eb';
          ctx2D.beginPath();
          ctx2D.arc(s.sx, s.sy, 2 * dpr, 0, Math.PI * 2);
          ctx2D.fill();

          // 节点名称标签
          if (view2D.scale > 25) {
            ctx2D.fillStyle = '#64748b';
            ctx2D.font = `${9 * dpr}px monospace`;
            ctx2D.textAlign = 'left';
            ctx2D.textBaseline = 'bottom';
            ctx2D.fillText(k, s.sx + 6 * dpr, s.sy - 3 * dpr);
          }
        });
      }

      // ------------------------------------------------------------
      // 4. 工位标注与对接位姿指示 (STATIONS)
      // ------------------------------------------------------------
      if (layerConfig.stations && telemetry && telemetry.scenario_metadata && telemetry.scenario_metadata.stations) {
        telemetry.scenario_metadata.stations.forEach(st => {
          const s = toScreen(st.x, st.y);
          const color = st.color || '#3b82f6';

          // 工位光晕底盘
          ctx2D.fillStyle = color;
          ctx2D.beginPath();
          ctx2D.arc(s.sx, s.sy, 9 * dpr, 0, Math.PI * 2);
          ctx2D.fill();
          ctx2D.strokeStyle = '#ffffff';
          ctx2D.lineWidth = 2 * dpr;
          ctx2D.stroke();

          // 对接航向指引箭头 (Docking Yaw Arrow)
          if (st.dock_yaw !== undefined) {
            const arrLen = 16 * dpr;
            const ax = s.sx + Math.cos(-st.dock_yaw) * arrLen;
            const ay = s.sy + Math.sin(-st.dock_yaw) * arrLen;
            ctx2D.strokeStyle = color;
            ctx2D.lineWidth = 3 * dpr;
            ctx2D.beginPath();
            ctx2D.moveTo(s.sx, s.sy);
            ctx2D.lineTo(ax, ay);
            ctx2D.stroke();
          }

          // 工位铭牌文字
          const txt = st.name || st.id;
          ctx2D.font = `bold ${10 * dpr}px sans-serif`;
          const tw = ctx2D.measureText(txt).width;
          ctx2D.fillStyle = 'rgba(255, 255, 255, 0.9)';
          ctx2D.fillRect(s.sx - tw / 2 - 4 * dpr, s.sy + 12 * dpr, tw + 8 * dpr, 14 * dpr);
          ctx2D.strokeStyle = '#cbd5e1';
          ctx2D.lineWidth = 1;
          ctx2D.strokeRect(s.sx - tw / 2 - 4 * dpr, s.sy + 12 * dpr, tw + 8 * dpr, 14 * dpr);

          ctx2D.fillStyle = '#0f172a';
          ctx2D.textAlign = 'center';
          ctx2D.textBaseline = 'middle';
          ctx2D.fillText(txt, s.sx, s.sy + 19 * dpr);
        });
      }

      // ------------------------------------------------------------
      // 5. 激光轮廓与扫描视场多边形 (LASER CONTOURS & FOV POLYGON)
      // ------------------------------------------------------------
      if (layerConfig.laser && telemetry && telemetry.scan_ranges && telemetry.scan_ranges.length > 0) {
        const ranges = telemetry.scan_ranges;
        const angleMin = telemetry.scan_angle_min !== undefined ? telemetry.scan_angle_min : -Math.PI;
        const angleInc = telemetry.scan_angle_inc !== undefined ? telemetry.scan_angle_inc : (Math.PI * 2 / ranges.length);
        const robotS = toScreen(deadReckonPose.x, deadReckonPose.y);

        const hitPoints = [];
        for (let i = 0; i < ranges.length; i++) {
          const r = ranges[i];
          if (r > 0.05 && r <= 15.0) {
            const beamAngle = deadReckonPose.yaw + angleMin + i * angleInc;
            const hx = deadReckonPose.x + Math.cos(beamAngle) * r;
            const hy = deadReckonPose.y + Math.sin(beamAngle) * r;
            const hs = toScreen(hx, hy);
            hitPoints.push({ sx: hs.sx, sy: hs.sy, dist: r });
          }
        }

        if (hitPoints.length > 2) {
          // 5.1 激光扫描覆盖扇区多边形 (Soft Green Coverage Wash)
          ctx2D.fillStyle = 'rgba(16, 185, 129, 0.07)';
          ctx2D.beginPath();
          ctx2D.moveTo(robotS.sx, robotS.sy);
          hitPoints.forEach(hp => {
            ctx2D.lineTo(hp.sx, hp.sy);
          });
          ctx2D.closePath();
          ctx2D.fill();

          // 5.2 激光边缘轮廓线 (Perimeter Contour Polyline with Safety Color Coding)
          for (let i = 0; i < hitPoints.length - 1; i++) {
            const pA = hitPoints[i];
            const pB = hitPoints[i + 1];
            // 若相邻两点距离合理，绘制轮廓线段
            const segDist = Math.hypot(pA.sx - pB.sx, pA.sy - pB.sy);
            if (segDist < 3.0 * view2D.scale) {
              const minDist = Math.min(pA.dist, pB.dist);
              if (minDist < 0.8) {
                ctx2D.strokeStyle = '#ef4444'; // 极近停机区：红色
                ctx2D.lineWidth = 3 * dpr;
              } else if (minDist < 1.4) {
                ctx2D.strokeStyle = '#f59e0b'; // 减速预警区：橙色
                ctx2D.lineWidth = 2.4 * dpr;
              } else {
                ctx2D.strokeStyle = '#10b981'; // 安全通行区：绿色
                ctx2D.lineWidth = 1.6 * dpr;
              }
              ctx2D.beginPath();
              ctx2D.moveTo(pA.sx, pA.sy);
              ctx2D.lineTo(pB.sx, pB.sy);
              ctx2D.stroke();
            }
          }

          // 5.3 激光击中点 (Laser Hit Dots)
          hitPoints.forEach((hp, idx) => {
            if (idx % 2 === 0) { // 适度降噪采样展示
              ctx2D.fillStyle = hp.dist < 0.8 ? '#ef4444' : (hp.dist < 1.4 ? '#f59e0b' : '#10b981');
              ctx2D.beginPath();
              ctx2D.arc(hp.sx, hp.sy, 2.2 * dpr, 0, Math.PI * 2);
              ctx2D.fill();
            }
          });
        }
      }

      // ------------------------------------------------------------
      // 6. 规划路径信息与流动导引 (PLANNED PATH & WAYPOINTS)
      // ------------------------------------------------------------
      const pathCard = document.getElementById('path-info-card');
      const hasPlan = telemetry && telemetry.plan_path && telemetry.plan_path.length > 1;

      if (layerConfig.path && hasPlan) {
        const path = telemetry.plan_path;

        // 更新右下角规划路径信息卡片
        if (pathCard) {
          pathCard.classList.remove('hidden');
          const pEng = document.getElementById('path-card-engine');
          const pWp = document.getElementById('path-card-waypoints');
          const pDist = document.getElementById('path-card-dist');
          const pGoal = document.getElementById('path-card-goal');
          const pStat = document.getElementById('path-card-status');

          if (pEng) pEng.textContent = (telemetry.planner_type === 'straight' ? '直连规划模式' : 'Dijkstra 拓扑导轨');
          if (pWp) pWp.textContent = `${path.length} 个航点`;
          if (pDist) pDist.textContent = `${(telemetry.nav_dist_rem || 0).toFixed(2)} m`;
          if (pGoal && telemetry.target_goal) {
            pGoal.textContent = `(${telemetry.target_goal.x.toFixed(1)}, ${telemetry.target_goal.y.toFixed(1)})`;
          }
          if (pStat) {
            const isNav = telemetry.nav_status === 'NAVIGATING';
            pStat.textContent = isNav ? '● 导航执行中' : '● 待命中';
            pStat.className = isNav
              ? 'px-1.5 py-0.2 rounded text-[10px] font-bold bg-blue-100 text-blue-800 font-mono animate-pulse'
              : 'px-1.5 py-0.2 rounded text-[10px] font-bold bg-slate-100 text-slate-700 font-mono';
          }
        }

        // 6.1 路径外发光底线 (Glowing Path Underlay)
        ctx2D.strokeStyle = 'rgba(147, 197, 253, 0.45)';
        ctx2D.lineWidth = 7 * dpr;
        ctx2D.lineCap = 'round';
        ctx2D.lineJoin = 'round';
        ctx2D.beginPath();
        path.forEach((pt, idx) => {
          const s = toScreen(pt.x, pt.y);
          if (idx === 0) ctx2D.moveTo(s.sx, s.sy);
          else ctx2D.lineTo(s.sx, s.sy);
        });
        ctx2D.stroke();

        // 6.2 动态流动虚线 (Flowing Dashed Path)
        ctx2D.strokeStyle = '#2563eb';
        ctx2D.lineWidth = 3.2 * dpr;
        const dashOffset = -(performance.now() / 25) % 20;
        ctx2D.setLineDash([8 * dpr, 6 * dpr]);
        ctx2D.lineDashOffset = dashOffset;
        ctx2D.beginPath();
        path.forEach((pt, idx) => {
          const s = toScreen(pt.x, pt.y);
          if (idx === 0) ctx2D.moveTo(s.sx, s.sy);
          else ctx2D.lineTo(s.sx, s.sy);
        });
        ctx2D.stroke();
        ctx2D.setLineDash([]); // 恢复实线

        // 6.3 航点圆圈与工步序号 (Waypoint Circles & Numbers)
        path.forEach((pt, idx) => {
          const s = toScreen(pt.x, pt.y);
          // 航点圆圈
          ctx2D.fillStyle = '#ffffff';
          ctx2D.beginPath();
          ctx2D.arc(s.sx, s.sy, 6 * dpr, 0, Math.PI * 2);
          ctx2D.fill();
          ctx2D.strokeStyle = '#2563eb';
          ctx2D.lineWidth = 2 * dpr;
          ctx2D.stroke();

          // 序号
          ctx2D.fillStyle = '#1e3a8a';
          ctx2D.font = `bold ${8 * dpr}px sans-serif`;
          ctx2D.textAlign = 'center';
          ctx2D.textBaseline = 'middle';
          ctx2D.fillText(String(idx + 1), s.sx, s.sy);
        });

        // 6.4 终点目标涟漪光晕 (Destination Beacon Ripple)
        const lastPt = path[path.length - 1];
        const goalS = toScreen(lastPt.x, lastPt.y);
        const rippleR = (12 + Math.sin(performance.now() / 180) * 5) * dpr;

        ctx2D.strokeStyle = 'rgba(239, 68, 68, 0.6)';
        ctx2D.lineWidth = 2 * dpr;
        ctx2D.beginPath();
        ctx2D.arc(goalS.sx, goalS.sy, rippleR, 0, Math.PI * 2);
        ctx2D.stroke();

        ctx2D.fillStyle = '#ef4444';
        ctx2D.beginPath();
        ctx2D.arc(goalS.sx, goalS.sy, 4 * dpr, 0, Math.PI * 2);
        ctx2D.fill();
      } else {
        if (pathCard && !pathCard.classList.contains('hidden')) {
          pathCard.classList.add('hidden');
        }
      }

      // ------------------------------------------------------------
      // 7. AMR 机器人车身底盘 (ROBOT CHASSIS & HEADING)
      // ------------------------------------------------------------
      const rS = toScreen(deadReckonPose.x, deadReckonPose.y);
      ctx2D.save();
      ctx2D.translate(rS.sx, rS.sy);
      ctx2D.rotate(-deadReckonPose.yaw); // Canvas Y 轴向下，取反

      const carW = 0.8 * view2D.scale;
      const carL = 1.2 * view2D.scale;

      // 车体底座投影发光
      ctx2D.fillStyle = 'rgba(37, 99, 235, 0.15)';
      ctx2D.fillRect(-carL / 2 - 4 * dpr, -carW / 2 - 4 * dpr, carL + 8 * dpr, carW + 8 * dpr);

      // 车身主矩形 (工业蓝)
      ctx2D.fillStyle = '#2563eb';
      ctx2D.fillRect(-carL / 2, -carW / 2, carL, carW);
      ctx2D.strokeStyle = '#1e3a8a';
      ctx2D.lineWidth = 2.5 * dpr;
      ctx2D.strokeRect(-carL / 2, -carW / 2, carL, carW);

      // 车头红三角航向标
      ctx2D.fillStyle = '#ef4444';
      ctx2D.beginPath();
      ctx2D.moveTo(carL / 2 + 12 * dpr, 0);
      ctx2D.lineTo(carL / 2 - 4 * dpr, -9 * dpr);
      ctx2D.lineTo(carL / 2 - 4 * dpr, 9 * dpr);
      ctx2D.closePath();
      ctx2D.fill();

      // 激光雷达发射原点 (翠绿)
      ctx2D.fillStyle = '#10b981';
      ctx2D.beginPath();
      ctx2D.arc(0, 0, 5 * dpr, 0, Math.PI * 2);
      ctx2D.fill();
      ctx2D.strokeStyle = '#ffffff';
      ctx2D.lineWidth = 1.5 * dpr;
      ctx2D.stroke();

      ctx2D.restore();
    }"""

code = code.replace(old_draw2d, new_draw2d)

with open("generate_console.py", "w", encoding="utf-8") as f:
    f.write(code)

print("generate_console.py successfully upgraded with 2D View enhancements!")
