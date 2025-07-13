const bagCommandTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/bag_command',
  messageType: 'std_msgs/String'
});

function populateRosbagTopics() {
  ros.getTopics((topics) => {
    const topicsDiv = document.getElementById('rosbag-topics');
    topicsDiv.innerHTML = '';
    topics.topics.forEach(topic => {
      if (topic.startsWith('/rosout') || topic.startsWith('/parameter_events')) return;
      const cb = document.createElement('input');
      cb.type = 'checkbox';
      cb.value = topic;
      cb.id = 'rosbag_cb_' + topic.replace(/\//g, '_');
      cb.checked = false;
      topicsDiv.appendChild(cb);
      topicsDiv.appendChild(document.createTextNode(' ' + topic));
      topicsDiv.appendChild(document.createElement('br'));
    });
    if (topicsDiv.innerHTML === '') topicsDiv.textContent = 'No topics found.';
    uiLog("[populateRosbagTopics] Topics:", topics.topics);
  });
}
populateRosbagTopics();
// setInterval(populateRosbagTopics, 30000);

let rosbagRecording = false;
function updateRosbagStatus() {
  const statusSpan = document.getElementById('rosbag-status');
  if (rosbagRecording) {
    statusSpan.innerHTML = 'Status: <span class="recording">Recording</span>';
    document.getElementById('rosbag-start-btn').disabled = true;
    document.getElementById('rosbag-stop-btn').disabled = false;
  } else {
    statusSpan.innerHTML = 'Status: <span class="idle">Idle</span>';
    document.getElementById('rosbag-start-btn').disabled = false;
    document.getElementById('rosbag-stop-btn').disabled = true;
  }
}
updateRosbagStatus();

function chooseFolder() {
  alert("Due to browser security, folder selection is not supported. Please type the path manually if you want to change it.");
}

function getSelectedRosbagTopics() {
  return Array.from(document.querySelectorAll('#rosbag-topics input[type=checkbox]:checked'))
    .map(cb => cb.value);
}

function startRosbagRecording() {
  const topics = getSelectedRosbagTopics();
  const dest = document.getElementById('dest-path').value;
  const filename = document.getElementById('filename').value.trim() || 'bag';
  if (topics.length === 0) {
    alert("Please select at least one topic to record.");
    return;
  }
  const command = {
    action: 'start',
    topics: topics,
    dest: dest,
    filename: filename
  };
  bagCommandTopic.publish({ data: JSON.stringify(command) });
  uiLog("[startRosbagRecording] Sent:", command);
  rosbagRecording = true;
  updateRosbagStatus();
}

function stopRosbagRecording() {
  const command = { action: 'stop' };
  bagCommandTopic.publish({ data: JSON.stringify(command) });
  uiLog("[stopRosbagRecording] Sent stop command");
  rosbagRecording = false;
  updateRosbagStatus();
}

const bagStatusTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/bag_status',
  messageType: 'std_msgs/String'
});
bagStatusTopic.subscribe(msg => {
  if (msg.data === 'recording') {
    rosbagRecording = true;
  } else {
    rosbagRecording = false;
  }
  updateRosbagStatus();
});

// --- Keep your original startRecording/stopRecording for cart control ---
function startRecording() {
  fetch('/start_recording').then(r => uiLog("Recording started"));
}
function stopRecording() {
  fetch('/stop_recording').then(r => uiLog("Recording stopped"));
}