# TDPS Smart Vehicle Project - Navigation Module
for the course UESTC3010: Team Design and Project Skills (TDPS)
### Team 13
### IMPORTANT NOTICE: THIS IS A COURSE PROJECT. ANY USES OF THE CODE THAT MIGHT CONSTITUDE PLAGIARISM FOR EITHER THIS COURSE OR ANY OTHER COURSES ARE NOT ALLOWED.
### Team Members

| **Name**         | **Group**          | **Responsibility**                                |
|------------------|--------------------|---------------------------------------------------|
| ZHANG Keyu       | Hardware           | **Team Leader** / *Hardware Group Manager*        |
| LIANG Sidi       | Vision             | **Project Manager** / *Vision Group Manager*      |
| DIAN Mingqian    | Control            | *Control Group Manager*                           |
| ZHANG Yuxin      | Hardware           |                                                   |
| HUANG Yukun      | Hardware           |                                                   |
| LIU Yicong       | Hardware           |                                                   |
| LI Chenjun       | Control            |                                                   |
| LIU Yi           | Control            |                                                   |
| YU Yunke         | Vision             |                                                   |
| SUN Qihao        | Vision             |                                                   |

The modules contained in this repository are built by Liang Sidi and Yu Yunke.


Important Files:
- main.py
- messaging.py 
- HCSR04
    - \_\_init\_\_.py
- Navigation
    - \_\_init\_\_.py
    - odometer.py


## Communication protocol：  
The specified parameters for UART communication to the main control board are as follows:

- Baudrate: 115200
- Bits: 8
- Parity: None
- Stop: 1

### Frame Structure

The communication protocol involves the exchange of frames, each consisting of specific byte sequences. The frames convey different messages based on their content. The meanings of key byte sequences are explained below:

1. **0xCC:** Acknowledgment (ACK)
2. **0xDD:** Retransmission Request

#### Message Content Format

The actual message content is structured as follows:

1. Start Marker: 0xAA, 0x55
2. Message Length
3. Message Content (in JSON format)

### Communication Sequence

#### Navigation Module to Master

1. Send: `0xCD`
2. Receive: `0xAA`, `0x55`, `{length of message n}`, `{message n}`, `0xCC` (if correct) or `0xDD` (if incorrect)
3. Send: `0xCC` (if correct) or `0xDD` (if incorrect)
4. Receive: `0xAA`, `0x55`, `{length of message m}`, `{message m}`, `0xCC` (if correct) or `0xDD` (if incorrect)
5. Send: `0xCC` (if correct) or `0xDD` (if incorrect)
6. Repeat the sequence...

| Sender | Receiver | Message Type | Byte Sequence                                             | Note  |
|--------|----------|--------------|-----------------------------------------------------------|------------------|
| Nav    | Master   | Control      | `0xCD`                                                  | Initiates the communication                |
| Master | Nav      | Data         | `0xAA, 0x55, {length of message n}, {message n}`         | - |
| Nav    | Master   | Acknowledgment | `0xCC (0xDD if incorrect)`                               | -                |
| Master | Nav      | Data         | `0xAA, 0x55, {length of message m}, {message m}`         | - |
| Nav    | Master   | Acknowledgment | `0xCC (0xDD if incorrect)`                               | -                |
| Master | Nav      | Control      | `0xCD`                                                  | Initiates another round of communication    |
| Nav    | Master   | Data         | `0xAA, 0x55, {length of message n+1}, {message n+1}`     | - |
| ...    | ...      | ...          | ...                                                       | ...              |


#### Message Content (4 ~ n)

The message content, ranging from the 4th byte onward, is expected to be in JSON format. Below are examples of the expected JSON structures for the OpenMV line-following section:

##### OpenMV Line Following (Sent by Navigation Module)

```json
{
  "Info_Task": 0,
  "Info_Patio": 1,
  "Info_Stage": 2,
  "Control_Angle": 45,
  "Control_Velocity": 10,
  "Control_PID": 3,
  "Control_Cam_Pitch": 1,
  "Control_Ball": 0,
  "Control_Comm": 1
}
```

##### Main Controller (Sent by Master)

```json
{
  "Info_Encoder_A": 123.45,
  "Info_Encoder_B": 67.89,
  "Info_Encoder_C": 101.23,
  "Info_Encoder_D": 54.32,
  "Info_Cam_Pitch": 1,
  "Info_Ball": 0,
  "Info_Comm": 1
}
```

