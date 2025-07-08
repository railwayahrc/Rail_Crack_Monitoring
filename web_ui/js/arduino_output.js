const arduinoOutputLines = document.getElementById('arduino-output-lines');
const cartSaysTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/cart_says',
  messageType: 'std_msgs/String'
});
cartSaysTopic.subscribe(msg => {
  const div = document.createElement('div');
  div.textContent = msg.data;
  arduinoOutputLines.appendChild(div);
  while (arduinoOutputLines.children.length > 20) {
    arduinoOutputLines.removeChild(arduinoOutputLines.firstChild);
  }
  arduinoOutputLines.scrollTop = arduinoOutputLines.scrollHeight;
});
