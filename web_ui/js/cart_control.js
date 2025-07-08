const armSwitch = document.getElementById('arm-switch');
const motorControls = document.getElementById('motor-controls');
const speedSlider = document.getElementById('speed');
const speedValue = document.getElementById('speed-value');
const reverseCheckbox = document.getElementById('reverse');
const light1Checkbox = document.getElementById('light1');
const light2Checkbox = document.getElementById('light2');

// ROS publisher for cart control
const cartControlTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/cart_control',
  messageType: 'std_msgs/String'
});

// Update speed value label
speedSlider.addEventListener('input', () => {
  speedValue.textContent = speedSlider.value;
  sendCartCommand();
});

// Send command whenever a control changes
[armSwitch, speedSlider, reverseCheckbox, light1Checkbox, light2Checkbox].forEach(el => {
  el.addEventListener('change', sendCartCommand);
});

function sendCartCommand() {
  const arm = armSwitch.checked ? 1 : 0;

  if (!arm) {
    motorControls.style.display = 'none';
    speedValue.textContent = '0';
    speedSlider.value = 0;
    reverseCheckbox.checked = false;
  } else {
    motorControls.style.display = '';
  }

  const speed = arm ? speedSlider.value : 0;
  const dir = arm ? (reverseCheckbox.checked ? 1 : 0) : 0;
  const l1 = light1Checkbox.checked ? 1 : 0;
  const l2 = light2Checkbox.checked ? 1 : 0;

  const cmd = `SPEED:${speed} DIR:${dir} L1:${l1} L2:${l2} ARM:${arm}`;
  cartControlTopic.publish({ data: cmd });
  uiLog("[cart_control] Sent:", cmd);
}

window.addEventListener('DOMContentLoaded', () => {
  motorControls.style.display = 'none';
  speedValue.textContent = speedSlider.value;
  sendCartCommand();
});
