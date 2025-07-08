function startRecording() {
  fetch('/start_recording').then(r => uiLog("Recording started"));
}
function stopRecording() {
  fetch('/stop_recording').then(r => uiLog("Recording stopped"));
}
