# FlightCtrl Simulator

![](https://g.gravizo.com/source/custom_mark10?https%3a%2f%2fraw%2egithubusercontent%2ecom%2fAkihiro%2dK%2fRasPiMain%2fmarker_simulator%2fFlightCtrlSimulator%2fREADME%2emd)
<details>
<summary></summary>
custom_mark10
  digraph G {
    subgraph cluster_0 {
      NC_comm[shape=box];
      label = "FlightCtrlSimulator";
    }
    subgraph cluster_1 {
      MarkerSimulator[shape=box];
      label = "MarkerSimulator";
    }
    subgraph cluster_2 {
      FC_comm[shape=box,label="ut_serial FC_comm"];
      marker_comm[shape=box,label="std::thread marker_comm"];
      label = "RasPiMain";
    }
    ttySIM0[label="/dev/ttySIM0"];
    ttyAMA0[label="/dev/ttyAMA0"];
    FC_comm -> ttyAMA0;
    ttyAMA0 -> FC_comm [label="Serial"];
    NC_comm -> ttySIM0;
    ttySIM0 -> NC_comm[label="Serial"];
    ttySIM0 -> ttyAMA0;
    ttyAMA0 -> ttySIM0[label="link with socat"];
    MarkerSimulator -> marker_comm[label="TCP"];
    marker_comm -> MarkerSimulator;
  }
custom_mark10
</details>

Prerequisites
--
 - socat: ``sudo apt-get install socat ``

How to use
--

In directory FlightCtrlSimulator/build,

1. Make program

  ```bash
  $ cmake ..
  $ make
  ```

2. Start socat and change permissions

  ```bash
  $ sudo socat -d -d pty,raw,echo=0,link=/dev/ttySIM0 pty,raw,echo=0,link=/dev/ttyAMA0

  ```

3. Open new terminal and change device permissions

  ``ctrl + shift + t``

  ```bash
  $ sudo chmod 666 /dev/ttySIM0
  $ sudo chmod 666 /dev/ttyAMA0
  ```

4. Run FlightCtrlSimulator

  ```bash
  $ ./flightctrl_simulator
  ```

5. Open new terminal and run MarkerSimulator (assuming it is already built)

  ``ctrl + shift + t``

  ```
  $ ../../MarkerSimulator/build/marker_simulator
  ```

6. Open new terminal and run RasPiMain

  ``ctrl + shift + t``

  ```
  $ cd ../../RasPiMain/build
  $ ./main
  ```

Todo
--
Tight coupling of FlightCtrlSimulator and MarkerSimulator.
Run a simulator of linear multicopter dynamics in the background and output simulated data with FlightCtrlSimulator and MarkerSimulator.


![](https://g.gravizo.com/source/custom_mark11?https%3a%2f%2fraw%2egithubusercontent%2ecom%2fAkihiro%2dK%2fRasPiMain%2fmarker_simulator%2fFlightCtrlSimulator%2fREADME%2emd)
<details>
<summary></summary>
custom_mark11
  digraph G {
    subgraph cluster_0 {
      subgraph cluster_1{
        NC_comm[shape=box];
        label = "FlightCtrlSimulator";
      }
      subgraph cluster_2{
        MarkerSimulator[shape=box,label="tcpserver MarkerSimulator"];
        label = "MarkerSimulator";
      }
      shared_memory[label="shared memory"];
      shared_memory -> NC_comm;
      shared_memory -> MarkerSimulator;
      label = "Multicopter Simulator";
    }
    subgraph cluster_3 {
      FC_comm[shape=box];
      marker_comm[shape=box, label ="std::thread marker_comm"];
      label = "RasPiMain";
    }
    ttySIM0[label="/dev/ttySIM0"];
    ttyAMA0[label="/dev/ttyAMA0"];
    FC_comm -> ttyAMA0;
    ttyAMA0 -> FC_comm [label="Serial"];
    NC_comm -> ttySIM0;
    ttySIM0 -> NC_comm[label="Serial"];
    ttySIM0 -> ttyAMA0;
    ttyAMA0 -> ttySIM0[label="link with socat"];
    MarkerSimulator -> marker_comm[label="TCP"];
    marker_comm -> MarkerSimulator;
  }
custom_mark11
</details>
