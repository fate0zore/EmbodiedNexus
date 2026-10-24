(() => {
  "use strict";

  const TOKEN_KEY = "embodiedNexusCoreToken";
  const REFRESH_MS = 1000;
  const TYPE_LABELS = {
    base: "底盘",
    arm: "机械臂",
    gripper: "夹爪",
    camera: "相机",
    lidar: "雷达",
  };
  const TYPE_ICONS = { base: "B", arm: "A", gripper: "G", camera: "C", lidar: "L" };
  const STATE_LABELS = {
    starting: "启动中",
    running: "运行中",
    stopped: "已停止",
    failed: "启动失败",
    stop_blocked: "停止受阻",
  };
  const byId = (id) => document.getElementById(id);
  const authView = byId("auth-view");
  const dashboard = byId("dashboard");
  const tokenForm = byId("token-form");
  const tokenInput = byId("token-input");
  const authError = byId("auth-error");
  const deviceGrid = byId("device-grid");
  const notice = byId("notice");
  let token = localStorage.getItem(TOKEN_KEY) || "";
  let refreshTimer = null;
  let refreshInFlight = false;
  let batchInFlight = false;
  let commandCatalog = null;
  let commandShell = "bash";
  let latestStatus = null;
  const deviceActionsInFlight = new Set();

  function showAuth(message = "") {
    if (refreshTimer) window.clearInterval(refreshTimer);
    refreshTimer = null;
    dashboard.hidden = true;
    authView.hidden = false;
    authError.textContent = message;
    authError.hidden = !message;
    tokenInput.focus();
  }

  function showDashboard() {
    authView.hidden = true;
    dashboard.hidden = false;
  }

  function setConnection(online, label) {
    const pill = byId("connection-pill");
    pill.classList.toggle("is-online", online);
    pill.classList.toggle("is-offline", !online);
    byId("connection-label").textContent = label;
    byId("core-state").classList.toggle("is-online", online);
    byId("core-state").classList.toggle("is-offline", !online);
    byId("core-state-label").textContent = online ? "运行中" : "不可达";
  }

  function setNotice(message, kind = "info") {
    notice.textContent = message;
    notice.className = `notice${kind === "error" ? " is-error" : kind === "warning" ? " is-warning" : ""}`;
    notice.hidden = !message;
  }

  async function apiRequest(path, options = {}) {
    const headers = new Headers(options.headers || {});
    headers.set("Authorization", `Bearer ${token}`);
    if (options.body) headers.set("Content-Type", "application/json");
    const response = await fetch(path, { ...options, headers, cache: "no-store" });
    const bodyText = await response.text();
    let payload = null;
    if (bodyText) {
      try { payload = JSON.parse(bodyText); } catch { payload = { detail: bodyText }; }
    }
    if (response.status === 401) {
      token = "";
      localStorage.removeItem(TOKEN_KEY);
      showAuth("Core token 无效或已失效，请重新输入。");
      const error = new Error("Core token 无效或已失效");
      error.status = 401;
      throw error;
    }
    if (!response.ok) {
      const detail = payload?.error || payload?.detail || `HTTP ${response.status}`;
      const error = new Error(detail);
      error.status = response.status;
      error.payload = payload;
      throw error;
    }
    return payload;
  }

  function element(tag, className, text) {
    const node = document.createElement(tag);
    if (className) node.className = className;
    if (text !== undefined) node.textContent = text;
    return node;
  }

  function readableState(value) {
    return STATE_LABELS[value] || value || "未知";
  }

  function readableBool(value) {
    if (value === true) return "是";
    if (value === false) return "否";
    return "未知";
  }

  function addFact(parent, label, value, positive = null) {
    const fact = element("div", "fact");
    fact.append(element("span", "fact-label", label));
    const valueNode = element("span", "fact-value", value);
    if (positive === true) valueNode.classList.add("is-positive");
    if (positive === false) valueNode.classList.add("is-negative");
    fact.append(valueNode);
    parent.append(fact);
  }

  function makeButton(label, className, action, deviceId = "", commandDeviceType = "") {
    const button = element("button", `button ${className}`, label);
    button.type = "button";
    button.dataset.focusTarget = deviceId ? `${deviceId}:${action}` : action;
    button.disabled = batchInFlight || (deviceId && deviceActionsInFlight.has(deviceId));
    button.addEventListener("click", () => {
      if (action === "commands") openCommandReference(commandDeviceType);
      else if (deviceId) runDeviceAction(deviceId, action);
      else runBatchAction(action);
    });
    return button;
  }

  function renderDevice(device) {
    const card = element("article", "device-card");
    const head = element("div", "device-head");
    const titleWrap = element("div", "device-title-wrap");
    titleWrap.append(element("span", "device-icon", TYPE_ICONS[device.type] || "D"));
    const name = element("div", "device-name");
    name.append(element("h3", "", device.id));
    name.append(element("span", "", TYPE_LABELS[device.type] || device.type || "设备"));
    titleWrap.append(name);
    const state = device.process_state || "unknown";
    const badge = element("span", `status-badge is-${state}`, readableState(state));
    head.append(titleWrap, badge);
    card.append(head);

    const facts = element("div", "device-facts");
    const deviceState = device.device_state || {};
    addFact(facts, "网关", readableBool(device.gateway_reachable), device.gateway_reachable);
    addFact(facts, "设备连接", readableBool(device.device_connected), device.device_connected);
    addFact(facts, "运动使能", readableBool(device.motion_enabled), device.motion_enabled);
    card.append(facts);

    if (device.last_error) card.append(element("p", "device-error", device.last_error));

    const footer = element("div", "device-footer");
    const actions = element("div", "device-actions");
    const running = ["running", "starting", "stop_blocked"].includes(state);
    if (!running) actions.append(makeButton("启动", "button-secondary", "start", device.id));
    if (running) {
      actions.append(makeButton("重启", "button-quiet", "restart", device.id));
      actions.append(makeButton("关闭", "button-danger", "shutdown", device.id));
    }
    actions.append(makeButton("命令", "button-quiet", "commands", device.id, device.type));
    footer.append(actions);
    footer.append(element("span", "last-seen", `最近响应 ${formatTime(device.last_seen_at)}`));
    card.append(footer);

    const details = element("details", "device-details");
    details.dataset.deviceId = device.id;
    details.append(element("summary", "", "设备状态详情"));
    details.append(element("pre", "", JSON.stringify({
      backend: deviceState.backend ?? null,
      device_connected: device.device_connected ?? null,
      motion_enabled: device.motion_enabled ?? null,
      last_error: device.last_error || "",
      device_state: deviceState,
    }, null, 2)));
    card.append(details);
    return card;
  }

  function formatTime(value) {
    if (!value) return "—";
    const date = new Date(value);
    return Number.isNaN(date.getTime()) ? String(value) : date.toLocaleTimeString();
  }

  function renderStatus(status) {
    latestStatus = status;
    setConnection(true, "Core 在线");
    if (notice.textContent.startsWith("正在连接 Core") ||
        notice.textContent.startsWith("读取 Core 状态失败")) setNotice("");
    byId("core-state-label").textContent = status.state === "running" ? "运行中" : status.state || "未知";
    byId("device-count").textContent = status.device_count ?? status.devices?.length ?? 0;
    byId("running-count").textContent = status.running_count ?? 0;
    byId("core-config-path").textContent = status.config_file || "平台配置已加载";
    byId("device-summary").textContent = `${status.devices?.length || 0} 个实例 · 自动刷新 ${REFRESH_MS / 1000}s`;
    byId("last-refresh").textContent = new Date().toLocaleTimeString();

    const openDetails = new Set([...deviceGrid.querySelectorAll("details[open]")]
      .map((details) => details.dataset.deviceId));
    const focusedTarget = document.activeElement?.dataset?.focusTarget || "";
    const devices = Array.isArray(status.devices) ? status.devices : [];
    deviceGrid.replaceChildren();
    if (devices.length === 0) {
      deviceGrid.append(element("div", "empty-state", "当前平台没有配置设备实例。"));
    } else {
      for (const device of devices) {
        const card = renderDevice(device);
        const details = card.querySelector("details");
        if (openDetails.has(device.id)) details.open = true;
        deviceGrid.append(card);
      }
    }
    if (focusedTarget) {
      const nextFocus = deviceGrid.querySelector(`[data-focus-target="${CSS.escape(focusedTarget)}"]`);
      if (nextFocus) nextFocus.focus({ preventScroll: true });
    }
  }

  async function refreshStatus() {
    if (!token || refreshInFlight) return;
    refreshInFlight = true;
    try {
      const status = await apiRequest("/api/v1/core/status");
      renderStatus(status);
    } catch (error) {
      if (error.status !== 401) {
        setConnection(false, "Core 不可达");
        setNotice(`读取 Core 状态失败：${error.message}`, "error");
      }
    } finally {
      refreshInFlight = false;
    }
  }

  function actionFailureMessage(payload) {
    const failures = (payload?.results || []).filter((result) => !result.success);
    if (failures.length) {
      return failures.map((result) => `${result.device_id}: ${result.error || result.device?.last_error || "操作失败"}`).join("；");
    }
    return payload?.error || "Core 未确认操作成功";
  }

  async function runDeviceAction(deviceId, action) {
    if (action === "shutdown" && !window.confirm(`关闭 ${deviceId} 服务？Core 会先请求安全停止，并等待反馈确认。`)) return;
    deviceActionsInFlight.add(deviceId);
    renderLastStatus();
    setNotice(`${deviceId}：正在${action === "start" ? "启动" : action === "restart" ? "重启" : "关闭"}…`);
    try {
      const result = await apiRequest(`/api/v1/core/devices/${encodeURIComponent(deviceId)}`, {
        method: "POST", body: JSON.stringify({ action }),
      });
      if (!result?.success) throw new Error(actionFailureMessage(result));
      setNotice(`${deviceId}：${action === "start" ? "启动" : action === "restart" ? "重启" : "关闭"}请求已完成。`);
    } catch (error) {
      if (error.status !== 401) setNotice(`${deviceId}：${error.message}`, "error");
    } finally {
      deviceActionsInFlight.delete(deviceId);
      await refreshStatus();
    }
  }

  async function runBatchAction(action) {
    if (batchInFlight) return;
    if (action === "shutdown_all" && !window.confirm("关闭所有已配置设备服务？Core 会逐个请求安全停止；未确认停稳的服务可能保留运行。")) return;
    batchInFlight = true;
    renderLastStatus();
    const label = action === "stop_all" ? "发送全部安全停止请求" : "关闭所有设备服务";
    setNotice(`${label}…`);
    try {
      const result = await apiRequest("/api/v1/core/devices/manage_all", {
        method: "POST", body: JSON.stringify({ action }),
      });
      if (!result?.success) throw new Error(actionFailureMessage(result));
      setNotice(action === "stop_all"
        ? "已发送全部停止请求；设备服务仍保持运行。"
        : "所有设备服务已关闭。");
    } catch (error) {
      if (error.status !== 401) setNotice(`${label}失败：${error.message}`, "error");
    } finally {
      batchInFlight = false;
      await refreshStatus();
    }
  }

  function renderLastStatus() {
    for (const button of document.querySelectorAll("button[data-focus-target]")) {
      const target = button.dataset.focusTarget;
      button.disabled = batchInFlight || [...deviceActionsInFlight].some((id) => target.startsWith(`${id}:`));
    }
    byId("stop-all-button").disabled = batchInFlight;
    byId("shutdown-all-button").disabled = batchInFlight;
  }

  async function openConfig() {
    const dialog = byId("config-dialog");
    const output = byId("config-output");
    output.textContent = "正在读取配置…";
    dialog.showModal();
    try {
      const config = await apiRequest("/api/v1/core/config");
      output.textContent = JSON.stringify(config, null, 2);
    } catch (error) {
      output.textContent = `配置读取失败：${error.message}`;
    }
  }

  async function loadCommandCatalog() {
    if (commandCatalog) return commandCatalog;
    const response = await fetch("/commands.json", { cache: "no-store" });
    if (!response.ok) throw new Error(`读取命令目录失败：HTTP ${response.status}`);
    commandCatalog = await response.json();
    return commandCatalog;
  }

  function replaceOptions(select, values, selectedValue = "") {
    select.replaceChildren();
    for (const item of values) {
      const option = document.createElement("option");
      option.value = item.value;
      option.textContent = item.label;
      select.append(option);
    }
    if (values.some((item) => item.value === selectedValue)) select.value = selectedValue;
  }

  function commandTargetValues(catalog) {
    return [
      { value: "core-mock", label: catalog.core.mock.label },
      { value: "core-operations", label: "Core 管理服务命令" },
      { value: "gazebo", label: catalog.gazebo.label },
      ...catalog.devices.map((device) => ({ value: `device:${device.type}`, label: `${device.label}独立节点` })),
    ];
  }

  function currentDeviceIds(catalog) {
    const configured = Array.isArray(latestStatus?.devices) ? latestStatus.devices : [];
    const configuredIds = configured.filter((device) => device.id).map((device) => ({
      value: device.id,
      label: `${TYPE_LABELS[device.type] || device.type || "设备"} · ${device.id}`,
    }));
    if (configuredIds.length) return configuredIds;
    const mockDefaults = { base: "base_main", arm: "arm_main", gripper: "gripper_main",
      camera: "camera_front", lidar: "lidar_main" };
    return catalog.devices.map((device) => ({
      value: mockDefaults[device.type] || `${device.type}_main`,
      label: `${TYPE_LABELS[device.type] || device.type} · ${mockDefaults[device.type] || `${device.type}_main`}`,
    }));
  }

  function selectedCommandOperation(catalog) {
    return catalog.core.operations.find((operation) => operation.id === byId("command-operation").value)
      || catalog.core.operations[0];
  }

  function selectedCommandDevice(catalog) {
    const selectedId = byId("command-device").value;
    const active = Array.isArray(latestStatus?.devices) ? latestStatus.devices : [];
    return active.find((device) => device.id === selectedId)?.id || selectedId || "base_main";
  }

  function buildCommand(catalog) {
    const target = byId("command-target").value;
    if (target === "core-mock") return catalog.core.mock.commands.join("\n");
    if (target === "gazebo") return catalog.gazebo.commands.join("\n");
    if (target === "core-operations") {
      const operation = selectedCommandOperation(catalog);
      const yamlDeviceId = selectedCommandDevice(catalog).replace(/'/g, "''");
      return operation.command.replace("{{device_id}}", yamlDeviceId);
    }
    if (target.startsWith("device:")) {
      const device = catalog.devices.find((item) => `device:${item.type}` === target);
      const profile = device?.profiles.find((item) => item.id === byId("command-profile").value)
        || device?.profiles[0];
      if (!device || !profile) return "未找到对应设备或配置。";
      return `ros2 launch ${device.package} ${device.launch} config_file:=${profile.config} interface_mode:=ros2 use_sim_time:=${profile.use_sim_time}`;
    }
    return "未找到对应命令。";
  }

  function quotePowerShell(value) {
    return `'${value.replace(/'/g, "''")}'`;
  }

  function renderCommandOutput(catalog) {
    const target = byId("command-target").value;
    const command = buildCommand(catalog);
    const targetInfo = target === "core-mock" ? catalog.core.mock
      : target === "gazebo" ? catalog.gazebo
        : target.startsWith("device:") ? catalog.devices.find((item) => `device:${item.type}` === target)
        : null;
    const operation = target === "core-operations" ? selectedCommandOperation(catalog) : null;
    const noteParts = [];
    if (targetInfo?.notes) noteParts.push(targetInfo.notes);
    if (operation && catalog.core.operations_note) noteParts.push(catalog.core.operations_note);
    if (operation?.needs_device) noteParts.push("设备 ID 来自当前 Core 状态；Core 尚未连接时使用 Mock 配置中的默认 ID。命令作用于 Core 当前加载的平台配置。");
    if (target.startsWith("device:") && catalog.device_notes) {
      const profileId = byId("command-profile").value;
      const profile = catalog.devices.find((item) => `device:${item.type}` === target)
        ?.profiles.find((item) => item.id === profileId);
      noteParts.push(catalog.device_notes.ownership);
      noteParts.push(profile?.id === "simulation" ? catalog.device_notes.simulation : catalog.device_notes.mock);
    }
    byId("command-note").textContent = noteParts.join(" ");

    if (commandShell === "powershell") {
      const distro = "Ubuntu-22.04";
      byId("command-output").textContent = [
        `$embodiedNexusWslDistro = ${quotePowerShell(distro)}`,
        "$embodiedNexusRepoWin = (Get-Location).Path -replace '\\\\', '/'",
        "if (-not (Test-Path (Join-Path $embodiedNexusRepoWin \"src\"))) { throw \"请先切换到 EmbodiedNexus 仓库根目录。\" }",
        "$embodiedNexusRepoWsl = (& wsl.exe -d $embodiedNexusWslDistro -- wslpath -u -a $embodiedNexusRepoWin).Trim()",
        "if ($LASTEXITCODE -ne 0 -or -not $embodiedNexusRepoWsl) { throw \"无法将当前仓库路径转换为 WSL 路径。\" }",
        `$embodiedNexusRosCommand = ${quotePowerShell(command)}`,
        '$embodiedNexusBashCommand = "cd \'$embodiedNexusRepoWsl\' && source /opt/ros/humble/setup.bash && source install/setup.bash && $embodiedNexusRosCommand"',
        "$embodiedNexusCommandBase64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($embodiedNexusBashCommand))",
        "$embodiedNexusLauncher = \"echo $embodiedNexusCommandBase64 | base64 -d | bash\"",
        "wsl.exe -d $embodiedNexusWslDistro -- bash -lc $embodiedNexusLauncher",
      ].join("\n");
    } else {
      byId("command-output").textContent = [
        "# 在仓库根目录的 Ubuntu / WSL Bash 中运行",
        "source /opt/ros/humble/setup.bash",
        "source install/setup.bash",
        command,
      ].join("\n");
    }
    byId("command-copy-status").textContent = "命令只会复制到剪贴板，不会执行。";
  }

  function refreshCommandSelectors(preferredTarget = "") {
    const catalog = commandCatalog;
    const targetSelect = byId("command-target");
    const previousTarget = preferredTarget || targetSelect.value;
    replaceOptions(targetSelect, commandTargetValues(catalog), previousTarget);
    const target = targetSelect.value;
    const profileField = byId("command-profile-field");
    const operationField = byId("command-operation-field");
    const deviceField = byId("command-device-field");
    profileField.hidden = !target.startsWith("device:");
    operationField.hidden = target !== "core-operations";
    const operationSelect = byId("command-operation");
    replaceOptions(operationSelect, catalog.core.operations.map((item) => ({ value: item.id, label: item.label })), operationSelect.value);
    const operation = selectedCommandOperation(catalog);
    deviceField.hidden = target !== "core-operations" || !operation.needs_device;
    replaceOptions(byId("command-device"), currentDeviceIds(catalog), byId("command-device").value);
    if (target.startsWith("device:")) {
      const device = catalog.devices.find((item) => `device:${item.type}` === target);
      const profileSelect = byId("command-profile");
      replaceOptions(profileSelect, (device?.profiles || []).map((item) => ({ value: item.id, label: item.label })), profileSelect.value || "mock");
    }
    renderCommandOutput(catalog);
  }

  async function openCommandReference(deviceType = "") {
    const dialog = byId("command-dialog");
    dialog.showModal();
    byId("command-output").textContent = "正在读取命令目录…";
    try {
      const catalog = await loadCommandCatalog();
      refreshCommandSelectors(deviceType ? `device:${deviceType}` : "");
    } catch (error) {
      byId("command-note").textContent = error.message;
      byId("command-output").textContent = "无法读取命令参考。";
    }
  }

  async function copyCommand() {
    const text = byId("command-output").textContent;
    try {
      if (navigator.clipboard?.writeText) {
        await navigator.clipboard.writeText(text);
      } else {
        const temporary = document.createElement("textarea");
        temporary.value = text;
        temporary.setAttribute("readonly", "");
        temporary.style.position = "fixed";
        temporary.style.opacity = "0";
        document.body.append(temporary);
        temporary.select();
        const copied = document.execCommand("copy");
        temporary.remove();
        if (!copied) throw new Error("浏览器拒绝访问剪贴板");
      }
      byId("command-copy-status").textContent = "已复制；请在对应终端检查后手动运行。";
    } catch (error) {
      byId("command-copy-status").textContent = `复制失败：${error.message}`;
    }
  }

  tokenForm.addEventListener("submit", async (event) => {
    event.preventDefault();
    token = tokenInput.value.trim();
    if (!token) return;
    localStorage.setItem(TOKEN_KEY, token);
    showDashboard();
    authError.hidden = true;
    tokenInput.value = "";
    setNotice("正在连接 Core…");
    await refreshStatus();
    if (!refreshTimer && token) refreshTimer = window.setInterval(refreshStatus, REFRESH_MS);
  });

  byId("refresh-button").addEventListener("click", refreshStatus);
  byId("stop-all-button").addEventListener("click", () => runBatchAction("stop_all"));
  byId("shutdown-all-button").addEventListener("click", () => runBatchAction("shutdown_all"));
  byId("config-button").addEventListener("click", openConfig);
  byId("commands-button").addEventListener("click", () => openCommandReference());
  byId("command-target").addEventListener("change", () => refreshCommandSelectors());
  byId("command-operation").addEventListener("change", () => refreshCommandSelectors());
  byId("command-device").addEventListener("change", () => commandCatalog && renderCommandOutput(commandCatalog));
  byId("command-profile").addEventListener("change", () => commandCatalog && renderCommandOutput(commandCatalog));
  for (const button of document.querySelectorAll("[data-command-shell]")) {
    button.addEventListener("click", () => {
      commandShell = button.dataset.commandShell;
      for (const tab of document.querySelectorAll("[data-command-shell]")) {
        const active = tab === button;
        tab.classList.toggle("is-active", active);
        tab.setAttribute("aria-pressed", String(active));
      }
      if (commandCatalog) renderCommandOutput(commandCatalog);
    });
  }
  byId("copy-command-button").addEventListener("click", copyCommand);
  byId("close-command-button").addEventListener("click", () => byId("command-dialog").close());
  byId("close-config-button").addEventListener("click", () => byId("config-dialog").close());
  byId("logout-button").addEventListener("click", () => {
    token = "";
    localStorage.removeItem(TOKEN_KEY);
    setNotice("");
    showAuth();
  });

  if (token) {
    showDashboard();
    refreshStatus().finally(() => {
      if (token) refreshTimer = window.setInterval(refreshStatus, REFRESH_MS);
    });
  } else {
    showAuth();
  }
})();
