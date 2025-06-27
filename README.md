# Ultra-wideband Real-time locating system

Authors: Michał Bielicki, Ekaterina Borodich, Mikołaj Janowski, Gabriela Jung

# System description

The system consists of a computer with Windows 11 operating system and seven devices (Nordic Semiconductors' nRF52840 DK and Qorvo’s DWM3000EVB UWB Module Arduino Shield) acting as: four anchors, two nodes and one receiver.

The system operation:

1. A node sends a poll message to every anchor and node and records the transmission time _poll_tx_.
2. An anchor/node receives a poll and responds with a final message containing the time of receiving the poll _poll_rx_ and the time of sending the response _final_tx_.
3. The node receives the final message and records the time _final_rx_.
4. The node calculates the distance to the other device:

$$d=\frac{((final_rx-poll_tx\ )-(final_tx-poll_rx\ ))}{2}\ \cdot\ c$$
c – speed of light

5. The node sends an “info” message to the receiver. The message contains: the source node address, the address of the device, distance to which was measured, and the distance between the devices.
6. The receiver outputs this data using Real Time Transfer in the following format:
```json
{
"from": (source node address),
"to": (other device's address),
"dist": (distance in centimeters)
}
```
7. The map application, running on a computer connected to the receiver via USB, reads the data and performs 2D multilateration of the source node. The position of the node marker on the map is updated whenever new distance data is available.

# User guide

1. Connect the nodes and anchors to a power supply and turn them on.
2. Connect the receiver to the computer with a USB cable that can transmit data.
3. Place anchors on the same height around the area where the nodes will be positioned.
4. Configure the anchor coordinates in the “data/anchor-config.json” file.
5. Run “main.py”
6. You can modify the anchors configuration by adding new anchors with the mouse wheel click, deleting with right mouse click, dragging them with the mouse, and editing the properties on the sidebar.

# Source code guide

The source code for the nRF and Map application can be found [here](https://tulodz-my.sharepoint.com/:f:/g/personal/247016_edu_p_lodz_pl/EhEbwTGUgANOl3lnzPNyBIYBDqWou5sJgB8XmJ2gHd1QLA?e=euhteq).

## NRF

The source code for the nRF devices can be found in the SEGGER Embedded Studio project “nRF\\dw-api\\Build_Platforms\\nRF52840-DK\\dw3000_api.emProject”.

The address of the programmed device has to be modified in “Source/examples/shared_data/shared_functions.h” at line 11.

### Receiver

In order to program the receiver, “#define TEST_DS_TWR_RESPONDER” has to be defined in the example selection file at "nRF\\dw-api\\Src\\example_selection.h". The code for the receiver is contained in “Source/examples/ex_05b_ds_twr_resp/ds_twr_responder.c”. The program waits for RX events to process incoming “info” messages. The RX events are handled in _rx_ok_cb()_ function.

### Nodes and anchors

In order to program the anchors and nodes, “#define TEST_DS_TWR_INITIATOR” has to be defined in the example selection file. The code for the anchors and nodes is contained in “Source/examples/ex_05a_ds_twr_init/ds_twr_initiator.c”.

The node functionality depends on the “#define DOES_POLL” definition in “Source/examples/shared_data/shared_functions.h” at line 10. Without this definition, the device will be programmed as an anchor.

The program waits for RX events to process incoming poll and final messages and sends polls in equal intervals if the node functionality is enabled. The polling takes place in a loop at line 370. The RX events are handled in _rx_ok_cb()_ function.

The delay after a series of polls to all anchors is defined at line 97. as NEW_COORDS_DLY.

The delay after a poll is defined at line 98 as _NEW_ANCHOR_DLY._

At line 61. the addresses of all the devices are defined.

## Map application

The map application uses Python 3.13.0 and is built upon Pygame library, the dependencies are listed in “requirements.txt”. The entry point of the application can be found in “map-app/main.py”. After initialization, the program waits for events, which are handled in “events.py” _process_events()_ function.

In “process_rtt.py” _start_rtt()_ function it can be chosen whether the data source should be the receiver connected via USB (use _rtt()_ generator) or a file of previously recorded data (use _file_rtt()_ generator). The receiver data can be recorded using the _log_info()_ function. The calculated node positions can be recorded with _log_node_position()_ method in “app.py”.

The node addresses are defined in “app.py” _update_nodes()_ method. First, the position of node 99 is calculated so that it can be considered in multilateration of node 98.

Multilateration takes place in “node.py” _multiliterate()_ method. The calculations are based on the latest distances data to all devices, which can amplify errors if the distance data is not accurate anymore due to a device being turned off or out of range.

# Accuracy test

This section presents the results of an experimental evaluation of a Real-Time Locating System (RTLS) utilizing Ultra-Wideband (UWB) technology. The system, comprising Nordic Semiconductors' nRF52840 microcontroller and Qorvo’s DW3000 UWB transceiver, was assessed in two distinct test configurations. The first test scenario involved a single node and four anchors, while the second test scenario consisted of two nodes and four anchors, with the position of only one node being examined.

In both trials, the nodes were systematically moved to a set of predefined reference points, allowing for a thorough analysis of the system's accuracy and reliability. Figures 1 and 2 show all calculated positions of the examined node together with their measured position as a reference.

| ![Fig 1](https://i.imgur.com/Nd2E9ji.png) | ![Fig 2](https://i.imgur.com/i6wQp0q.png) |
| --- | --- |
| Fig. 1. Calculated node positions for Test 1 | Fig. 2. Calculated node positions for Test 2 |

Figure 3 presents the errors of the multilateration over time, for both tests. It can be observed that the errors are similar at the beginning of both tests. It could be because the position of the second node was not yet calculated and therefore not considered in multilateration.

| ![Fig 3](https://i.imgur.com/sNEVQhp.png) |
| --- |
| Fig. 3. Error over time |

Table 1 presents the distribution of error for both tests. It can be seen that the error in Test 2 is greater and less predictable than the error in Test 1.

| Test 1: Without the second node | Test 2: With the second node |
| --- | --- |
| ![Table 1a](https://i.imgur.com/WTKZT89.png) | ![Table 1b](https://i.imgur.com/jH009uG.png) |
| MSE: 553.60<br><br>ME: 20.85<br><br>Variance: 118.98<br><br>Standard deviation: 10.91 | MSE: 31354.37<br><br>ME: 163.40<br><br>Variance: 4654.19<br><br>Standard deviation: 68.22 |
| Table 1. Distribution of error |     |

The accuracy of node positioning was significantly lower when the second node was introduced. This may be caused by the fact that the multilateration of the examined node included the calculated position of the second node which was encumbered with some error, causing the errors to accumulate in the examined node’s calculated location.

# Team members’ responsibilities

| Team member | Responsibilities |
| --- | --- |
| Michał Bielicki | Communication between nodes, ranging. |
| Ekaterina Borodich | Map application GUI. |
| Mikołaj Janowski | Multilateration. Research on BLE. |
| Gabriela Jung | Reading data from the receiver via USB. Accuracy test and analysis of results. |
