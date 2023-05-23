# vehicle_creation

This package includes three main components, the availability publisher, priority publisher and the vehicle manager node.
The availability publisher publishes the vehicle's availability onto the MQTT broker while the priority publisher publishes the vehicle's priority.

The vehicle manager node provides a `vehicle` class to manage each UGV's topics and subscribers. It republishes messages from multiple UGVs onto a single topic for other nodes to subscribe to. It also offers simple premptive collision avoidence.
