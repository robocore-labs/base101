const svcList = document.getElementById('service-list');
const logsSection = document.getElementById('logs');
const logsOutput = document.getElementById('logs-output');
const logsServiceEl = document.getElementById('logs-service');
let logSource = null;

async function fetchServices() {
  try {
    const res = await fetch('/api/services');
    const services = await res.json();
    renderServices(services);
  } catch (err) {
    svcList.textContent = `Failed to load services: ${err}`;
  }
}

function renderServices(services) {
  svcList.innerHTML = '';
  if (!services.length) {
    svcList.textContent = 'No services found for this compose file.';
    return;
  }
  const table = document.createElement('table');
  table.innerHTML = '<thead><tr><th>Service</th><th>State</th><th>Image</th><th>Actions</th></tr></thead>';
  const tbody = document.createElement('tbody');
  for (const svc of services) {
    const name = svc.Service || svc.Name || svc.service || '';
    const state = svc.State || svc.Status || '';
    const image = svc.Image || '';
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${name}</td><td>${state}</td><td>${image}</td><td class="actions"></td>`;
    const actions = tr.querySelector('.actions');
    for (const action of ['start', 'stop', 'restart', 'update']) {
      const btn = document.createElement('button');
      btn.textContent = action;
      btn.onclick = () => runAction(name, action);
      actions.appendChild(btn);
    }
    const logsBtn = document.createElement('button');
    logsBtn.textContent = 'logs';
    logsBtn.onclick = () => openLogs(name);
    actions.appendChild(logsBtn);
    tbody.appendChild(tr);
  }
  table.appendChild(tbody);
  svcList.appendChild(table);
}

async function runAction(name, action) {
  const res = await fetch(`/api/services/${encodeURIComponent(name)}/${action}`, { method: 'POST' });
  if (!res.ok) {
    const body = await res.json().catch(() => ({}));
    alert(`${action} ${name} failed: ${body.detail || res.statusText}`);
  }
  fetchServices();
}

function openLogs(name) {
  if (logSource) logSource.close();
  logsOutput.textContent = '';
  logsServiceEl.textContent = name;
  logsSection.hidden = false;
  logSource = new EventSource(`/api/services/${encodeURIComponent(name)}/logs`);
  logSource.onmessage = (evt) => {
    const line = JSON.parse(evt.data);
    logsOutput.textContent += line + '\n';
    logsOutput.scrollTop = logsOutput.scrollHeight;
  };
  logSource.onerror = () => {
    logsOutput.textContent += '\n[log stream closed]\n';
  };
}

document.getElementById('logs-close').onclick = () => {
  if (logSource) logSource.close();
  logSource = null;
  logsSection.hidden = true;
};

document.getElementById('refresh').onclick = fetchServices;

// --- config editor ---
const configTree = document.getElementById('config-tree');
const editor = document.getElementById('editor');
const editorPath = document.getElementById('editor-path');
const saveBtn = document.getElementById('save');
const saveStatus = document.getElementById('save-status');
let currentPath = null;

async function fetchConfigTree() {
  const res = await fetch('/api/config/tree');
  const paths = await res.json();
  configTree.innerHTML = '';
  for (const path of paths) {
    const li = document.createElement('li');
    li.textContent = path;
    li.onclick = () => loadConfig(path);
    configTree.appendChild(li);
  }
}

async function loadConfig(path) {
  const res = await fetch(`/api/config/file?path=${encodeURIComponent(path)}`);
  if (!res.ok) {
    const body = await res.json().catch(() => ({}));
    alert(`load failed: ${body.detail || res.statusText}`);
    return;
  }
  const body = await res.json();
  currentPath = path;
  editorPath.textContent = path;
  editor.value = body.content;
  editor.disabled = false;
  saveBtn.disabled = false;
  saveStatus.textContent = '';
}

saveBtn.onclick = async () => {
  if (!currentPath) return;
  saveStatus.textContent = 'saving…';
  const res = await fetch('/api/config/file', {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ path: currentPath, content: editor.value }),
  });
  if (res.ok) {
    saveStatus.textContent = 'saved — restart the owning service above to apply';
  } else {
    const body = await res.json().catch(() => ({}));
    saveStatus.textContent = `error: ${body.detail || res.statusText}`;
  }
};

fetchServices();
fetchConfigTree();
setInterval(fetchServices, 5000);
