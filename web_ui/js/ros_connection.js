const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
ros.on('connection', () => uiLog('✅ Connected to rosbridge'));
ros.on('error', err => uiLog('❌ ROS connection error:', err));
ros.on('close', () => uiLog('❌ ROS connection closed'));

const viewers = [1, 2, 3];
const imgTags = ["viewer1", "viewer2", "viewer3"];
const topicSelects = ["topic1", "topic2", "topic3"];

viewers.forEach(i => {
  document.getElementById(`enableCam${i}`).addEventListener('change', () => updateViewer(i));
  document.getElementById(`topic${i}`).addEventListener('change', () => updateViewer(i));
});

function updateViewer(i) {
  const enabled = document.getElementById(`enableCam${i}`).checked;
  const topic = document.getElementById(`topic${i}`).value;
  const img = document.getElementById(`viewer${i}`);
  img.src = enabled && topic ? `http://localhost:8080/stream?topic=${topic}` : "";
  uiLog(`[updateViewer] viewer${i}: enabled=${enabled}, topic=${topic}, img.src=${img.src}`);
}

function fetchImageTopics() {
  ros.getTopics((topics) => {
    const imageTopics = topics.topics.filter((t, idx) => topics.types[idx] === 'sensor_msgs/msg/Image');
    viewers.forEach(i => {
      const select = document.getElementById(`topic${i}`);
      select.innerHTML = '';
      imageTopics.forEach(topic => {
        const opt = document.createElement('option');
        opt.value = topic;
        opt.text = topic;
        select.appendChild(opt);
      });
      updateViewer(i);
    });
    uiLog("[fetchImageTopics] Found image topics:", imageTopics);
  });
}

fetchImageTopics();
