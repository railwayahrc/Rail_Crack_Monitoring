const freqMonitorToggle = document.getElementById('freq-monitor-toggle');
const freqMonitorClient = new ROSLIB.Service({
  ros,
  name: '/toggle_frequency_monitor',
  serviceType: 'std_srvs/SetBool'
});

freqMonitorToggle.addEventListener('change', function() {
  freqMonitorClient.callService(
    { data: freqMonitorToggle.checked },
    function(result) {
      uiLog(`[freqMonitorToggle] Monitoring ${result.success ? 'enabled' : 'disabled'}: ${result.message}`);
      if (!freqMonitorToggle.checked) {
        document.getElementById('topic-freqs').textContent = "Monitoring disabled.";
      }
    }
  );
});

// Camera parameter and exposure control
function setCameraParam(camIndex, paramName, value, type = 'string') {
  const payload = {
    cam_index: camIndex,
    param: paramName,
    value: value.toString()
  };
  uiLog("[setCameraParam] Backend payload:", payload);

  fetch("http://localhost:8001/set_param", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload)
  })
  .then(async response => {
    let data;
    try {
      data = await response.json();
    } catch (e) {
      data = {
        error: "Invalid JSON in response",
        status: response.status,
        text: await response.text()
      };
    }
    return data;
  })
  .then(result => {
    uiLog("[setCameraParam] Backend response:", result);
    if (result.success) {
      uiLog(`[setCameraParam] SUCCESS setting ${paramName} for camera_${camIndex}`);
    } else {
      uiLog(`[setCameraParam] ERROR setting ${paramName} for camera_${camIndex}:`, result.error || result.detail || result.text);
    }
  })
  .catch(error => {
    uiLog("[setCameraParam] Network or backend error:", error);
  });
}

viewers.forEach(i => {
  const toggle = document.getElementById(`exposureToggle${i}`);
  const slider = document.getElementById(`exposureSlider${i}`);
  const topicSelector = document.getElementById(`topic${i}`);

  function getCamIndexFromTopic() {
    const topic = topicSelector.value || '';
    const match = topic.match(/camera_(\d+)/);
    if (match) {
      uiLog(`[getCamIndexFromTopic] Camera index for viewer${i}:`, match[1]);
      return parseInt(match[1]);
    } else {
      uiLog(`[getCamIndexFromTopic] No camera index found for viewer${i}, topic:`, topic);
      return null;
    }
  }

  function updateExposureUI() {
    if (toggle.checked) {
      slider.style.display = 'none';
      slider.disabled = true;
      uiLog(`[UI] viewer${i}: Auto Exposure checked, hiding slider`);
    } else {
      slider.style.display = 'block';
      slider.disabled = false;
      uiLog(`[UI] viewer${i}: Auto Exposure unchecked, showing slider`);
    }
  }
  updateExposureUI();

  toggle.addEventListener('change', () => {
    const camIndex = getCamIndexFromTopic();
    if (camIndex === null) {
      uiLog(`[toggle.change] No valid topic selected for camera ${i}`);
      updateExposureUI();
      return;
    }

    if (toggle.checked) {
      slider.style.display = 'none';
      slider.disabled = true;
      uiLog(`[toggle.change] viewer${i}: Auto Exposure ENABLED, sending exposure_mode=auto`);
      setCameraParam(camIndex, 'exposure_mode', 'auto', 'string');
    } else {
      slider.style.display = 'block';
      slider.disabled = false;
      uiLog(`[toggle.change] viewer${i}: Auto Exposure DISABLED, sending exposure_mode=manual`);
      setCameraParam(camIndex, 'exposure_mode', 'manual', 'string');
      uiLog(`[toggle.change] viewer${i}: Setting exposure_value=${slider.value}`);
      setCameraParam(camIndex, 'exposure_value', slider.value, 'integer');
    }
  });

  slider.addEventListener('input', () => {
    const camIndex = getCamIndexFromTopic();
    if (camIndex !== null && !toggle.checked) {
      uiLog(`[slider.input] viewer${i}: Manual mode, setting exposure_value=${slider.value}`);
      setCameraParam(camIndex, 'exposure_value', slider.value, 'integer');
    } else {
      uiLog(`[slider.input] viewer${i}: Ignored slider event (auto mode or no cam index)`);
    }
  });

  topicSelector.addEventListener('change', () => {
    uiLog(`[topicSelector.change] viewer${i}: Topic changed to`, topicSelector.value);
    updateExposureUI();
  });
});
