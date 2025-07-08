new ROSLIB.Topic({
  ros,
  name: '/battery_percent',
  messageType: 'std_msgs/String'
}).subscribe(msg => document.getElementById('battery-percent').textContent = msg.data);

new ROSLIB.Topic({
  ros,
  name: '/system_time',
  messageType: 'std_msgs/String'
}).subscribe(msg => document.getElementById('system-time').textContent = msg.data);

new ROSLIB.Topic({
  ros,
  name: '/disk_usage',
  messageType: 'std_msgs/String'
}).subscribe(msg => document.getElementById('disk-usage').textContent = msg.data);

new ROSLIB.Topic({
  ros,
  name: '/topic_frequencies',
  messageType: 'std_msgs/String'
}).subscribe(msg => document.getElementById('topic-freqs').textContent = msg.data);
