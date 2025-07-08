// On-page logging system
function uiLog(...args) {
  const panel = document.getElementById('ui-log-panel');
  const msg = args.map(a =>
    typeof a === 'object' ? JSON.stringify(a, null, 2) : a
  ).join(' ');
  panel.innerHTML += `<div>${msg}</div>`;
  panel.scrollTop = panel.scrollHeight;
  if (typeof console !== "undefined" && console.log) {
    console.log(...args);
  }
}

window.onerror = function(message, url, line, column, error) {
  uiLog("JS ERROR:", message, "at", url + ":" + line + ":" + column, error);
};
window.onunhandledrejection = function(event) {
  uiLog("UNHANDLED PROMISE REJECTION:", event.reason);
};
["click", "change", "input"].forEach(type => {
  window.addEventListener(type, function(e) {
    uiLog(`[EVENT] ${type} on`, e.target.id || e.target.className || e.target.tagName);
  }, true);
});
